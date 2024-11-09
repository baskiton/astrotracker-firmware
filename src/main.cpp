#include <settings.h>

#include <GyverHub.h>
#include <WiFi.h>

#include <driver/timer.h>

// .env
#define XSTR(x) #x
#define STR(x) F(XSTR(x))


void sec_to_comp(double sec, int &d, int &m, double &s) {
    double dd = sec / 3600.0;
    d = (int)dd;
    m = (int)((dd - d) * 60);
    s = (dd - d - m / 60.0) * 3600.0;
}

void sec_to_comp(double sec, int &d, int &m, int &s) {
    sec_to_comp(sec, d, m, (double &)s);
}

int comp_to_sec(int d, int m, int s) {
    int sign = (d < 0 || m < 0 || s < 0)? -1: 1;
    return sign * (d * 3600 + m * 60 + s);
}

double racomp_to_rasec(uint8_t h, uint8_t m, double s) {
    return h * 3600 + m * 60 + s;
}

double ra_to_asec(double ra) {
    return ra * 15.0;
}

double asec_to_ra(double asec) {
    return asec / 15.0;
}

void calculate_stepper_timer(double sps, uint16_t &divider, uint64_t &alarm_value) {
    double x = infinity();
    for (int presc = 2; presc < 65536; ++presc) {
        double ticks = APB_CLK_FREQ / (sps * presc);
        double xx = abs((ticks * presc) - (double)((uint64_t)ticks * presc)) / APB_CLK_FREQ;
        if (xx < x) {
            x = xx;
            divider = presc;
            alarm_value = (uint64_t)ticks;
        }
    }
}

// from esp-hal-timer.c
struct hw_timer_s {
    uint8_t group;
    uint8_t num;
};

typedef enum {
    IDLE = 0,
    SHOT,
    INTV,
} shutter_state;

class AstroTracker;
bool IRAM_ATTR stepper_timer_isr(AstroTracker *tracker);
bool IRAM_ATTR shutter_timer_isr(AstroTracker *tracker);

class AstroTracker {

public:
    int64_t steps = 0;
    int64_t move_steps = 0;
    uint64_t position_steps = 0;

    AstroTracker(int step_pin, int dir_pin, int direction = 1, int shutter_pin = -1, int en_pin = -1) :
            _step_pin(step_pin),
            _dir_pin(dir_pin),
            _en_pin(en_pin),
            _shutter_pin(shutter_pin) {
        _cur_dir = _direction = (direction > 0) ? 1 : -1;

        pinMode(step_pin, OUTPUT);
        digitalWrite(step_pin, 0);
        pinMode(dir_pin, OUTPUT);
        digitalWrite(dir_pin, _direction > 0);

        if (en_pin != -1) {
            pinMode(en_pin, OUTPUT);
            digitalWrite(en_pin, 1);
        }
        if (shutter_pin != -1) {
            pinMode(shutter_pin, OUTPUT);
            digitalWrite(shutter_pin, 0);
        }

        init_sidereal_timer();
    }

    void enable(bool val) {
         if (_en_pin != -1)
             digitalWrite(_en_pin, !val);
    }

    void start_sidereal(bool reload = true) {
        _sideric = true;
        if (reload)
            steps = 0;
        reverse(false);
        digitalWrite(_step_pin, 0);
        timerRestart(_sidereal_timer);
        timerAlarmEnable(_sidereal_timer);
    }

    void stop_sidereal(bool pause = false) {
        timerAlarmDisable(_sidereal_timer);
        digitalWrite(_step_pin, 0);
        if (!pause)
            _sideric = false;
    }

    void reverse(bool val) {
        digitalWrite(_dir_pin, (_direction > 0) ^ val);
        _cur_dir = ((_direction > 0) ^ val)? 1: -1;
    }

    void step() {
        digitalWrite(_step_pin, 1);
        delayMicroseconds(STEPPER_HIGH_TIME_US);
        digitalWrite(_step_pin, 0);
        steps += _cur_dir;
        if (move_steps) {
            if (_cur_dir > 0)
                position_steps = (position_steps - 1) > (uint64_t)STEPS_PER_REV? ((uint64_t)STEPS_PER_REV - 1): (position_steps - 1);
            else
                position_steps = (position_steps + 1) % (uint64_t)STEPS_PER_REV;
            if (!(move_steps -= _cur_dir))
                stop_move();
        }
    }

    void half_step() {
        static int x = 0;
        digitalWrite(_step_pin, (x ^= 1));
        if (x) {
            steps += _cur_dir;
            if (move_steps) {
                if (_cur_dir > 0)
                    position_steps = (position_steps - 1) > (uint64_t)STEPS_PER_REV? ((uint64_t)STEPS_PER_REV - 1): (position_steps - 1);
                else
                    position_steps = (position_steps + 1) % (uint64_t)STEPS_PER_REV;
                if (!(move_steps -= _cur_dir))
                    stop_move();
            }
        }
    }

    bool start_shutter(int shots, int shot_intv, int shot_exposure, double shot_delay_s) {
        bool ret = false;
        if (shots && shot_intv >= 0 && shot_exposure > 0) {
            _shots = shots;
            _shot_intv = shot_intv;
            _shot_exposure = shot_exposure;
            _shot_state = SHOT;
            ret = init_shutter_timer(shot_delay_s);
        }
        return ret;
    }

    void stop_shutter() {
        end_shutter_timer();
        _shots = 0;
        _shot_state = IDLE;
    }

    void _shot() {
        switch (_shot_state) {
        case SHOT:  // shot done
            if (!(--_shots))
                stop_shutter();
            else {
                timerWrite(_shutter_timer, _shot_intv);
                digitalWrite(_shutter_pin, 0);
                _shot_state = INTV;
            }
            break;

        case INTV:  // interval done
            timerRestart(_shutter_timer);
            digitalWrite(_shutter_pin, 1);
            _shot_state = SHOT;
            break;

        default:
            break;
        }
    }

    void shutter_status(int &shots, shutter_state &state) {
        shots = _shots;
        state = _shot_state;
    }

    void stop_move(bool restart = true) {
        end_mover_timer();
        move_steps = 0;
        reverse(false);
        if (restart && _sideric)
            start_sidereal(false);
    }

    void move(int64_t _steps) {
        if (!_steps)
            return;
        if (_sideric)
            stop_sidereal(true);
        end_mover_timer();
        digitalWrite(_step_pin, 0);
        move_steps += _steps;
        reverse(move_steps < 0);
        if (move_steps) {
            init_mover_timer();
            timerAlarmEnable(_mover_timer);
        }
    }

    void move_to(int64_t _pos) {
        if (_pos == position_steps)
            return;
        if (_sideric)
            stop_sidereal(true);
        stop_move(false);

        int64_t diff = position_steps - _pos;
        if (abs(diff) > (STEPS_PER_REV / 2))
            diff += (diff < 0)? STEPS_PER_REV: -STEPS_PER_REV;
        move(diff);
    }

private:
    int _step_pin;
    int _dir_pin;
    int _en_pin;
    int _direction;     // -1, 1
    int _cur_dir;

    bool _sideric = false;
    int _shutter_pin;
    int _shots = 0;
    int _shot_intv = 0;
    int _shot_exposure = 0;
    shutter_state _shot_state = IDLE;

    hw_timer_t *_sidereal_timer = nullptr;
    hw_timer_t *_mover_timer = nullptr;
    hw_timer_t *_shutter_timer = nullptr;

    void init_sidereal_timer() {
        init_stepper_timer(&_sidereal_timer, SIDER_TMR, SPS);
    }

    void init_mover_timer() {
        if (!_mover_timer)
            init_stepper_timer(&_mover_timer, MOVE_TMR, MOVE_SPS);
    }

    void init_stepper_timer(hw_timer_t **timer, uint8_t tmr_n, double sps, int intr_alloc_flags = 0) {
        uint16_t divider;
        uint64_t alarm_value;
        calculate_stepper_timer(sps, divider, alarm_value);
        hw_timer_t *t = *timer = timerBegin(tmr_n, divider, true);
        timer_isr_callback_add((timer_group_t)t->group, (timer_idx_t)t->num,
                                (timer_isr_t)stepper_timer_isr, this, intr_alloc_flags);
        timerAlarmWrite(t, alarm_value, true);
        timerAlarmDisable(t);
    }

    void end_mover_timer() {
        if (_mover_timer) {
            digitalWrite(_step_pin, 0);
            timerAlarmDisable(_mover_timer);
            timerDetachInterrupt(_mover_timer);
            timerEnd(_mover_timer);
            _mover_timer = nullptr;
        }
    }

    bool init_shutter_timer(double shot_delay_s) {
        bool ret = false;
        if (_shutter_pin != -1 && !_shutter_timer) {
            _shutter_timer = timerBegin(SHUT_TMR, SHUTTER_DIV, true);
            timer_isr_callback_add((timer_group_t)_shutter_timer->group, (timer_idx_t)_shutter_timer->num, (timer_isr_t)shutter_timer_isr, this, ESP_INTR_FLAG_LEVEL1);
            _shot_exposure = (int)((_shot_exposure + shot_delay_s) * SHUTTER_MUL);
            _shot_intv = _shot_exposure - _shot_intv * SHUTTER_MUL;
            timerAlarmWrite(_shutter_timer, _shot_exposure, true);
            timerAlarmEnable(_shutter_timer);
            digitalWrite(_shutter_pin, 1);  // start the first capture
            ret = true;
        }
        return ret;
    }

    void end_shutter_timer() {
        if (_shutter_pin != -1 && _shutter_timer) {
            digitalWrite(_shutter_pin, 0);
            timerAlarmDisable(_shutter_timer);
            timerDetachInterrupt(_shutter_timer);
            timerEnd(_shutter_timer);
            _shutter_timer = nullptr;
        }
    }
};

bool IRAM_ATTR stepper_timer_isr(AstroTracker *tracker) {
    tracker->step();

    // some additional logic or handling may be required here to approriately yield or not
    return false;
}

bool IRAM_ATTR shutter_timer_isr(AstroTracker *tracker) {
    tracker->_shot();

    // some additional logic or handling may be required here to approriately yield or not
    return false;
}


class Interface {

public:
    void init(AstroTracker *tracker) {
        _tracker = tracker;

        // подключение к WiFi..
# ifdef TRACKER_WIFI_AP
        WiFi.mode(WIFI_AP);
        WiFi.softAP(STR(SSID_AP), STR(PASS_AP));
# else
        WiFi.mode(WIFI_STA);
        WiFi.begin(STR(SSID_TO), STR(PASS_TO));
        while (WiFi.status() != WL_CONNECTED) {
            delay(500);
            Serial.print(".");
        }
        Serial.print(F("\nConnected: "));
        Serial.println(WiFi.localIP());
# endif

        Serial.print(F("ID: "));
        Serial.println(_hub.id);

        _hub.config(F("AstroTracker"), F("ESP32"), F("f753"));
        _hub.onBuild([this](gh::Builder& b) { this->on_build(b); });
        _hub.onUnix([this](uint32_t stamp) { this->on_unix(stamp); });
        _hub.begin();
    }

    void tick(uint64_t us) {
        _hub.tick();

        if ((us - _upd_tmr_us) >= _upd_intv_us) {
            _upd_tmr_us = us;
            if (_do_run)
                _runned_us = us;

            // main
            uint64_t t = (_runned_us - _run_time_us) / 1000000L;
            gh::Update upd(&_hub);
            #ifdef _SHOW_US
            upd.update(F("us")).value(_runned_us - _run_time_us);
            #endif
            upd.update(F("time")).value(t);
            upd.update(F("steps")).value(_tracker->steps);
            upd.update(F("sky_ra")).value(asec_to_ra(_tracker->position_steps * SEC_PER_STEP));
            double s;
            int d, m;
            sec_to_comp(_tracker->position_steps * SEC_PER_STEP, d, m, s);
            snprintf(_sky_deg, sizeof(_sky_deg), "%3i° %2i' %6.03f\"", d, m, s);
            upd.update(F("sky_deg")).value(_sky_deg);
            // main-movement
            sec_to_comp(_tracker->move_steps * SEC_PER_STEP, d, m, s);
            upd.update(F("move_rem_ra")).value(asec_to_ra(_tracker->move_steps * SEC_PER_STEP));
            snprintf(_move_rem_deg, sizeof(_move_rem_deg), "%3i° %2i' %6.03f\"", d, abs(m), abs(s));
            upd.update(F("move_rem_deg")).value(_move_rem_deg);

            // shutter
            int shots;
            shutter_state state;
            _tracker->shutter_status(shots, state);
            upd.update(F("capt_status_shots")).value((shots > 0)? shots: -shots);
            upd.update(F("capt_status_state")).value(F((state == IDLE)? "IDLE":
                                                       (state == SHOT)? "SHOT":
                                                       (state == INTV)? "INTV":
                                                       "INVALID"));
            upd.send();
            if (state == IDLE) {
                _do_exposure = false;
                capt_upd();
            }
        }
    }

private:
    GyverHub _hub;
    AstroTracker *_tracker = nullptr;
    uint64_t _upd_intv_us = 1000000;
    uint64_t _upd_tmr_us = 0;
    uint64_t _run_time_us = 0;
    uint64_t _runned_us = 0;
    bool _do_run = false;
    int _images = 0;
    int _exposure_s = CAMERA_EXPOSURE;
    int _intv_s = CAMERA_INTV;
    double _shot_delay_s = CAMERA_SHOT_DELAY;
    bool _do_exposure = false;
    char _sky_deg[32]{};
    char _move_rem_deg[32]{};

    // movement
    uint8_t _move_h = 0, _move_m = 0;
    double _move_s = 0.0;

    void on_build(gh::Builder& b) {
        static byte tab;
        if (b.Tabs(&tab).text(F("Main;Capture;Network")).click())
            b.refresh();

        b.show(tab == 0);   // Main
        if (b.Confirm_(F("main_conf")).text(F(_do_run? "Stop tracking?": "Start tracking?")).click()) {
            int x = b.build.value;
            if (!_do_run && x) {
                _runned_us = _run_time_us = esp_timer_get_time() + 33333 * 2;
                _tracker->start_sidereal();
                Serial.println(F("Run motor!"));
            } else if (x) {
                _tracker->stop_sidereal();
                _runned_us = esp_timer_get_time();
                Serial.println(F("Stop motor!"));
            }
            _do_run ^= x;
            gh::Update upd(&_hub);
            upd.update(F("main_conf")).text(F(_do_run? "Stop tracking?": "Start tracking?"));
            upd.update(F("main_but")).icon(F(_do_run? "f28d": "f144"));
            upd.send();
        }
        b.Button_(F("main_but")).label(F("Run")).icon(F(_do_run? "f28d": "f144"))
            .attach([this]() { this->_hub.sendAction(F("main_conf")); });
        #ifdef _SHOW_US
        b.Label_(F("us")).value(0).label(F("us")).fontSize(18);
        #endif
        b.beginRow();
        b.Time_(F("time")).value(0).disabled().size(3).fontSize(18);
        b.Label_(F("steps")).value(0).label(F("steps")).size(7).fontSize(18);
        b.endRow();
        b.beginRow();
        b.Time_(F("sky_ra")).value(0).label(F("RA")).disabled().size(3).fontSize(18);
        b.Label_(F("sky_deg")).value(0).label(F("DEG")).disabled().size(7).fontSize(18);
        b.endRow();

        b.Title(F("Movement"));
        b.beginRow();
        b.Spinner_(F("move_h"), &_move_h).range(0, 23, 1).label(F("RA, h")).size(3);
        b.Spinner_(F("move_m"), &_move_m).range(0, 59, 1).label(F("RA, m")).size(3);
        b.Spinner_(F("move_s"), &_move_s).range(0.0, 60.0 - RA_SEC_PER_STEP, RA_SEC_PER_STEP, 3).label(F("RA, s")).size(4);
        b.endRow();
        b.beginRow();
        b.Time_(F("move_rem_ra")).value(0).label(F("RA remaining")).disabled().size(3);
        b.Label_(F("move_rem_deg")).value(0).label(F("DEG remaining")).disabled().size(7).fontSize(18);
        b.endRow();
        b.beginRow();
        b.Button_(F("move_bwd")).noLabel().icon(F("f2ea")).size(3).attach([this]() {
            this->_tracker->move(-llround(ra_to_asec(racomp_to_rasec(this->_move_h, this->_move_m, this->_move_s)) / SEC_PER_STEP));
        });
        b.Button_(F("move_stop")).noLabel().icon(F("f04d")).size(4).attach([this]() {
            this->_tracker->stop_move();
        });
        b.Button_(F("move_fwd")).noLabel().icon(F("f2f9")).size(3).attach([this]() {
            this->_tracker->move(llround(ra_to_asec(racomp_to_rasec(this->_move_h, this->_move_m, this->_move_s)) / SEC_PER_STEP));
        });
        b.endRow();
        b.beginRow();
        b.Button_(F("move_to")).label(F("move to")).icon(F("f78c")).size(1).attach([this]() {
            this->_tracker->move_to(llround(ra_to_asec(racomp_to_rasec(this->_move_h, this->_move_m, this->_move_s)) / SEC_PER_STEP));
        });
        b.Button_(F("move_set")).label(F("set current")).icon(F("f111")).size(1).attach([this]() {
            this->_tracker->position_steps = llround(ra_to_asec(racomp_to_rasec(this->_move_h, this->_move_m, this->_move_s)) / SEC_PER_STEP);
        });
        b.endRow();

        b.show(tab == 1);   // Capture
        b.beginRow();
        b.Time_(F("capt_exp"), &_exposure_s).label(F("Exposure time")).disabled(_do_exposure);
        b.Time_(F("capt_intv"), &_intv_s).label(F("Time between capture")).disabled(_do_exposure);
        b.endRow();
        b.beginRow();
        b.Input_(F("capt_delay"), &_shot_delay_s).label(F("Shot delay, s")).disabled(_do_exposure);
        b.Input_(F("capt_images"), &_images).label(F("Images to capture")).disabled(_do_exposure).hint(F("-1 - infinite* mode"));
        b.endRow();
        if (b.Confirm_(F("capt_conf")).text(F(_do_exposure? "Stop shuttering?": "Start shuttering?")).click()) {
            int x = b.build.value;
            Serial.print(F("Exposure "));
            if (!_do_exposure && x && _images && _intv_s >= 0 && _exposure_s > 0) {
                Serial.println(F("Begin!"));
                if ((_intv_s < CAMERA_MIN_INTV))
                    _intv_s = CAMERA_MIN_INTV;
                _tracker->start_shutter(_images, _intv_s, _exposure_s, _shot_delay_s);
            } else if (x) {
                Serial.println(F("Stop!"));
                _tracker->stop_shutter();
            }
            _do_exposure ^= x;
            capt_upd();
        }
        b.Button_(F("capt_but"))
            .color(_do_exposure? gh::Colors::Blue: gh::Colors::Red)
            .label(F("Capture"))
            .attach([this]() { this->_hub.sendAction(F("capt_conf")); });
        b.Title(F("Status"));
        b.beginRow();
        b.Label_(F("capt_status_shots")).label(F("shots remaining")).value(0);
        b.Label_(F("capt_status_state")).label(F("state"));
        b.endRow();

        b.show(tab == 2);   // Network
        b.Switch().label(F("AP"));
        b.Input().label(F("SSID"));
        b.Pass();
        b.Button().label(F("Connect"));

        b.show();
    }

    void capt_upd() {
        gh::Update upd(&_hub);
        upd.update(F("capt_images")).disabled(_do_exposure);
        upd.update(F("capt_exp")).valueVar(&_exposure_s).disabled(_do_exposure);
        upd.update(F("capt_intv")).valueVar(&_intv_s).disabled(_do_exposure);
        upd.update(F("capt_delay")).disabled(_do_exposure);
        upd.update(F("capt_conf")).text(F(_do_exposure? "Stop shuttering": "Start shuttering"));
        upd.update(F("capt_but")).color(_do_exposure? gh::Colors::Blue: gh::Colors::Red).label(F(_do_exposure? "Stop": "Capture"));
        upd.send();
    }

    void on_unix(uint32_t stamp) {
    }
};


AstroTracker tracker(STEP_S_PIN, STEP_D_PIN, 1, SHUTTER_PIN, STEP_E_PIN);
Interface iface;

void setup() {
    Serial.begin(115200);
    Serial.println(F("setup"));
    iface.init(&tracker);
    tracker.enable(true);
}

void loop() {
    iface.tick(esp_timer_get_time());
}
