#include <settings.h>

#include <GyverHub.h>

#include <driver/timer.h>

// .env
#define XSTR(x) #x
#define STR(x) F(XSTR(x))


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

    AstroTracker(int step_pin, int dir_pin, int direction = 1, int shutter_pin = -1, int en_pin = -1) :
            _step_pin(step_pin),
            _dir_pin(dir_pin),
            _en_pin(en_pin),
            _shutter_pin(shutter_pin) {
        _direction = (direction > 0) ? 1 : -1;

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

        init_stepper_timer();
    }

    void enable(bool val) {
        if (_en_pin != -1)
            digitalWrite(_en_pin, !val);
        if (!val) {
            timerAlarmDisable(_stepper_timer);
        } else {
            steps = 0;
            timerRestart(_stepper_timer);
            timerAlarmEnable(_stepper_timer);
        }
    }

    void reverse(bool val) {
        digitalWrite(_dir_pin, (_direction > 0) ^ val);
    }

    void step() {
        digitalWrite(_step_pin, 1);
        delayMicroseconds(STEPPER_HIGH_TIME_US);
        digitalWrite(_step_pin, 0);
        steps += _direction;
    }

    void half_step() {
        static int x = 0;
        digitalWrite(_step_pin, (x ^= 1));
        steps += _direction * x;
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

private:
    int _step_pin;
    int _dir_pin;
    int _en_pin;
    int _direction;     // -1, 1

    int _shutter_pin;
    int _shots;
    int _shot_intv = 0;
    int _shot_exposure = 0;
    shutter_state _shot_state = IDLE;

    hw_timer_t *_stepper_timer = nullptr;
    hw_timer_t *_shutter_timer = nullptr;

    void init_stepper_timer() {
        uint16_t divider;
        uint64_t alarm_value;
        calculate_stepper_timer(SPS * 2, divider, alarm_value);
        _stepper_timer = timerBegin(0, divider, true);
        timer_isr_callback_add((timer_group_t)_stepper_timer->group, (timer_idx_t)_stepper_timer->num, (timer_isr_t)stepper_timer_isr, this, ESP_INTR_FLAG_LEVEL3);
        timerAlarmWrite(_stepper_timer, alarm_value, true);
        timerAlarmDisable(_stepper_timer);
    }

    void calculate_stepper_timer(double half_sps, uint16_t &divider, uint64_t &alarm_value) {
        double x = infinity();
        for (int presc = 2; presc < 65536; ++presc) {
            double ticks = APB_CLK_FREQ / (half_sps * presc);
            double y = ticks - int(ticks);
            if (y < x) {
                x = y;
                divider = presc;
                alarm_value = ticks;
            }
        }
    }

    bool init_shutter_timer(double shot_delay_s) {
        bool ret = false;
        if (_shutter_pin != -1 && !_shutter_timer) {
            _shutter_timer = timerBegin(1, SHUTTER_DIV, true);
            timer_isr_callback_add((timer_group_t)_shutter_timer->group, (timer_idx_t)_shutter_timer->num, (timer_isr_t)shutter_timer_isr, this, ESP_INTR_FLAG_LEVEL1);
            _shot_exposure = (_shot_exposure + shot_delay_s) * SHUTTER_MUL;
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
    tracker->half_step();

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
        // _tracker->tick(us);
        _hub.tick();

        if ((us - _upd_tmr_us) >= _upd_intv_us) {
            _upd_tmr_us = us;
            if (_do_run)
                _runned_us = us;

            uint64_t t = (_runned_us - _run_time_us) / 1000000L;
            gh::Update upd(&_hub);
            upd.update(F("us")).value(_runned_us - _run_time_us);
            upd.update(F("time")).value(t);
            upd.update(F("steps")).value(_tracker->steps);
            double dd = _tracker->steps / STEPS_PER_DEG;
            int d = (int)dd;
            int m = (int)((dd - d) * 60);
            double s = (dd - d - m / 60.0) * 3600.0;
            upd.update(F("sky_deg")).value(d);
            upd.update(F("sky_min")).value(m);
            upd.update(F("sky_sec")).value(s);
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

    void on_build(gh::Builder& b) {
        static byte tab;
        if (b.Tabs(&tab).text(F("Main;Capture;Network")).click())
            b.refresh();

        b.show(tab == 0);   // Main
        if (b.Confirm_(F("main_conf")).text(F(_do_run? "Stop tracking?": "Start tracking?")).click()) {
            int x = b.build.value;
            if (!_do_run && x) {
                _runned_us = _run_time_us = esp_timer_get_time() + 33333 * 2;
                _tracker->enable(1);
                Serial.println(F("Run motor!"));
            } else if (x) {
                _tracker->enable(0);
                _runned_us = esp_timer_get_time();
                Serial.println(F("Stop motor!"));
            }
            _do_run ^= x;
            gh::Update upd(&_hub);
            upd.update(F("main_conf")).text(F(_do_run? "Stop tracking?": "Start tracking?"));
            upd.update(F("main_but")).icon(F(_do_run? "f28d": "f144"));
            upd.send();
        }
        b.Button_(F("main_but"))
            .label(F("Run"))
            .icon(F(_do_run? "f28d": "f144"))
            .attach([this](){ this->_hub.sendAction(F("main_conf")); });
        b.Label_(F("us")).value(0).label(F("us"));
        b.beginRow();
        b.Time_(F("time")).value(0).disabled();
        b.Label_(F("steps")).value(0).label(F("steps"));
        b.endRow();
        b.beginRow();
        b.Label_(F("sky_deg")).value(0).label(F("Sky degrees"));
        b.Label_(F("sky_min")).value(0).label(F("Sky minutes"));
        b.Label_(F("sky_sec")).value(0.0).label(F("Sky seconds"));
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
            .attach([this](){ this->_hub.sendAction(F("capt_conf")); });
        // add status
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
}

void loop() {
    iface.tick(esp_timer_get_time());
}
