#include <settings.h>
#include <Arduino.h>
#include <driver/timer.h>

#include <FS.h>
#include <GyverHub.h>
#include <LittleFS.h>
#include <WiFi.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>

#include "catalogues.hpp"


void on_wifi_sta_ip(arduino_event_t *event) {
//    event->event_info.wifi_sta_connected;
    Serial.print(F("\nConnected: "));
    Serial.println(WiFi.localIP());
}

void on_wifi_sta_dis(arduino_event_t *event) {
    auto e = (wifi_err_reason_t)event->event_info.wifi_sta_disconnected.reason;
    auto *s = F("UNKNOWN");
    switch (e) {
    case WIFI_REASON_UNSPECIFIED:
        s = F("UNSPECIFIED");
        break;
    case WIFI_REASON_AUTH_EXPIRE:
        s = F("AUTH_EXPIRE");
        break;
    case WIFI_REASON_AUTH_LEAVE:
        s = F("AUTH_LEAVE");
        break;
    case WIFI_REASON_ASSOC_EXPIRE:
        s = F("ASSOC_EXPIRE");
        break;
    case WIFI_REASON_ASSOC_TOOMANY:
        s = F("ASSOC_TOOMANY");
        break;
    case WIFI_REASON_NOT_AUTHED:
        s = F("NOT_AUTHED");
        break;
    case WIFI_REASON_NOT_ASSOCED:
        s = F("NOT_ASSOCED");
        break;
    case WIFI_REASON_ASSOC_LEAVE:
        s = F("ASSOC_LEAVE");
        break;
    case WIFI_REASON_ASSOC_NOT_AUTHED:
        s = F("ASSOC_NOT_AUTHED");
        break;
    case WIFI_REASON_DISASSOC_PWRCAP_BAD:
        s = F("DISASSOC_PWRCAP_BAD");
        break;
    case WIFI_REASON_DISASSOC_SUPCHAN_BAD:
        s = F("DISASSOC_SUPCHAN_BAD");
        break;
    case WIFI_REASON_BSS_TRANSITION_DISASSOC:
        s = F("BSS_TRANSITION_DISASSOC");
        break;
    case WIFI_REASON_IE_INVALID:
        s = F("IE_INVALID");
        break;
    case WIFI_REASON_MIC_FAILURE:
        s = F("MIC_FAILURE");
        break;
    case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
        s = F("4WAY_HANDSHAKE_TIMEOUT");
        break;
    case WIFI_REASON_GROUP_KEY_UPDATE_TIMEOUT:
        s = F("GROUP_KEY_UPDATE_TIMEOUT");
        break;
    case WIFI_REASON_IE_IN_4WAY_DIFFERS:
        s = F("IE_IN_4WAY_DIFFERS");
        break;
    case WIFI_REASON_GROUP_CIPHER_INVALID:
        s = F("GROUP_CIPHER_INVALID");
        break;
    case WIFI_REASON_PAIRWISE_CIPHER_INVALID:
        s = F("PAIRWISE_CIPHER_INVALID");
        break;
    case WIFI_REASON_AKMP_INVALID:
        s = F("AKMP_INVALID");
        break;
    case WIFI_REASON_UNSUPP_RSN_IE_VERSION:
        s = F("UNSUPP_RSN_IE_VERSION");
        break;
    case WIFI_REASON_INVALID_RSN_IE_CAP:
        s = F("INVALID_RSN_IE_CAP");
        break;
    case WIFI_REASON_802_1X_AUTH_FAILED:
        s = F("802_1X_AUTH_FAILED");
        break;
    case WIFI_REASON_CIPHER_SUITE_REJECTED:
        s = F("CIPHER_SUITE_REJECTED");
        break;
    case WIFI_REASON_TDLS_PEER_UNREACHABLE:
        s = F("TDLS_PEER_UNREACHABLE");
        break;
    case WIFI_REASON_TDLS_UNSPECIFIED:
        s = F("TDLS_UNSPECIFIED");
        break;
    case WIFI_REASON_SSP_REQUESTED_DISASSOC:
        s = F("SSP_REQUESTED_DISASSOC");
        break;
    case WIFI_REASON_NO_SSP_ROAMING_AGREEMENT:
        s = F("NO_SSP_ROAMING_AGREEMENT");
        break;
    case WIFI_REASON_BAD_CIPHER_OR_AKM:
        s = F("BAD_CIPHER_OR_AKM");
        break;
    case WIFI_REASON_NOT_AUTHORIZED_THIS_LOCATION:
        s = F("NOT_AUTHORIZED_THIS_LOCATION");
        break;
    case WIFI_REASON_SERVICE_CHANGE_PERCLUDES_TS:
        s = F("SERVICE_CHANGE_PERCLUDES_TS");
        break;
    case WIFI_REASON_UNSPECIFIED_QOS:
        s = F("UNSPECIFIED_QOS");
        break;
    case WIFI_REASON_NOT_ENOUGH_BANDWIDTH:
        s = F("NOT_ENOUGH_BANDWIDTH");
        break;
    case WIFI_REASON_MISSING_ACKS:
        s = F("MISSING_ACKS");
        break;
    case WIFI_REASON_EXCEEDED_TXOP:
        s = F("EXCEEDED_TXOP");
        break;
    case WIFI_REASON_STA_LEAVING:
        s = F("STA_LEAVING");
        break;
    case WIFI_REASON_END_BA:
        s = F("END_BA");
        break;
    case WIFI_REASON_UNKNOWN_BA:
        s = F("UNKNOWN_BA");
        break;
    case WIFI_REASON_TIMEOUT:
        s = F("TIMEOUT");
        break;
    case WIFI_REASON_PEER_INITIATED:
        s = F("PEER_INITIATED");
        break;
    case WIFI_REASON_AP_INITIATED:
        s = F("AP_INITIATED");
        break;
    case WIFI_REASON_INVALID_FT_ACTION_FRAME_COUNT:
        s = F("INVALID_FT_ACTION_FRAME_COUNT");
        break;
    case WIFI_REASON_INVALID_PMKID:
        s = F("INVALID_PMKID");
        break;
    case WIFI_REASON_INVALID_MDE:
        s = F("INVALID_MDE");
        break;
    case WIFI_REASON_INVALID_FTE:
        s = F("INVALID_FTE");
        break;
    case WIFI_REASON_TRANSMISSION_LINK_ESTABLISH_FAILED:
        s = F("TRANSMISSION_LINK_ESTABLISH_FAILED");
        break;
    case WIFI_REASON_ALTERATIVE_CHANNEL_OCCUPIED:
        s = F("ALTERATIVE_CHANNEL_OCCUPIED");
        break;
    case WIFI_REASON_BEACON_TIMEOUT:
        s = F("BEACON_TIMEOUT");
        break;
    case WIFI_REASON_NO_AP_FOUND:
        s = F("NO_AP_FOUND");
        break;
    case WIFI_REASON_AUTH_FAIL:
        s = F("AUTH_FAIL");
        break;
    case WIFI_REASON_ASSOC_FAIL:
        s = F("ASSOC_FAIL");
        break;
    case WIFI_REASON_HANDSHAKE_TIMEOUT:
        s = F("HANDSHAKE_TIMEOUT");
        break;
    case WIFI_REASON_CONNECTION_FAIL:
        s = F("CONNECTION_FAIL");
        break;
    case WIFI_REASON_AP_TSF_RESET:
        s = F("AP_TSF_RESET");
        break;
    case WIFI_REASON_ROAMING:
        s = F("ROAMING");
        break;
    case WIFI_REASON_ASSOC_COMEBACK_TIME_TOO_LONG:
        s = F("ASSOC_COMEBACK_TIME_TOO_LONG");
        break;
    case WIFI_REASON_SA_QUERY_TIMEOUT:
        s = F("SA_QUERY_TIMEOUT");
        break;
    }
    Serial.print(F("WiFi disconnect: "));
    Serial.println(s);
//    Serial.println("Trying to reconnect...");
//    WiFi.reconnect();
//    WiFi.begin(_sta_ssid, _sta_pass);
}

void sec_to_comp(double sec, int &d, int &m, double &s, bool signing = false) {
    if (isnan(sec)) {
        s = d = m = 0;
        return;
    }
    double dd = sec / 3600.0;
    d = (int)dd;
    m = (int)((dd - d) * 60);
    s = (dd - d - m / 60.0) * 3600.0;
    if (signing) {
        int sign = (d < 0 || m < 0 || s < 0)? -1 : 1;
        d = sign * abs(d);
        m = abs(m);
        s = abs(s);
    }
}

void sec_to_comp(double sec, int &d, int &m, int &s, bool signing = false) {
    sec_to_comp(sec, d, m, (double &)s, signing);
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

#ifdef WITH_DECMPU
class DecMpu {
public:
    float ypr[3]{};   // yaw(Z), pitch(Y), roll(X)

    void init() {
        _mpu.initialize();
        int16_t offsets[6]{};

        File f = LittleFS.open(F("/mpu_offsets.dat"), "r");
        if (f) {
            f.read((uint8_t *)offsets, sizeof(offsets));
            f.close();
            _mpu.setXAccelOffset(offsets[0]);
            _mpu.setYAccelOffset(offsets[1]);
            _mpu.setZAccelOffset(offsets[2]);
            _mpu.setXGyroOffset(offsets[3]);
            _mpu.setYGyroOffset(offsets[4]);
            _mpu.setZGyroOffset(offsets[5]);
            Serial.println("Set reads:");
            _mpu.PrintActiveOffsets();
        }

        // DMP init
        if (!_mpu.dmpInitialize()) {
            _mpu.setDMPEnabled(true);
            Serial.println(F("DMP inited"));
        }
    }

    void calibrate() {
        Serial.println("\ncalibrate");
        // load default accuracy
        _mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
        _mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

        // reset offsets
        _mpu.setXAccelOffset(0);
        _mpu.setYAccelOffset(0);
        _mpu.setZAccelOffset(0);
        _mpu.setXGyroOffset(0);
        _mpu.setYGyroOffset(0);
        _mpu.setZGyroOffset(0);

        _mpu.CalibrateAccel();
        _mpu.CalibrateGyro();

        int16_t offsets[6] = {
            _mpu.getXAccelOffset(),
            _mpu.getYAccelOffset(),
            _mpu.getZAccelOffset(),
            _mpu.getXGyroOffset(),
            _mpu.getYGyroOffset(),
            _mpu.getZGyroOffset(),
        };

        File f = LittleFS.open(F("/mpu_offsets.dat"), "w");
        if (f) {
            f.write((uint8_t *)offsets, sizeof(offsets));
            f.close();
            Serial.println("Offsets writes");
        }
        Serial.println("Set offsets:");
        _mpu.PrintActiveOffsets();

        // DMP init
        _mpu.dmpInitialize();
        _mpu.setDMPEnabled(true);
    }

    void tick() {
        if (!_mpu.dmpGetCurrentFIFOPacket(_fifo_buf))
            return;

        _mpu.dmpGetQuaternion(&_q, _fifo_buf);

        Quaternion q_tmp = _q_base.getProduct(_q);
        VectorFloat gravity;
        _mpu.dmpGetGravity(&gravity, &q_tmp);
        _mpu.dmpGetYawPitchRoll(ypr, &q_tmp, &gravity);
    }

    void set_base(bool reset = false) {
        if (reset)
            _q_base = {1,0,0,0};
        else {
            // inverse
            float n = _q.w*_q.w + _q.x*_q.x + _q.y*_q.y + _q.z*_q.z;    // norm
            _q_base = {_q.w / n, -_q.x / n, -_q.y / n, -_q.z / n};
        }
    }

private:
    MPU6050 _mpu;
    Quaternion _q{}, _q_base{};
    uint8_t _fifo_buf[45]{};
};
#endif

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

    bool start_shutter(int shots, int shot_intv, int shot_exposure, double shot_delay_s, int dither_dist, int dither_shots) {
        bool ret = false;
        if (shots && shot_intv >= 0 && shot_exposure > 0) {
            _shots = shots;
            _shot_intv = shot_intv;
            _shot_exposure = shot_exposure;
            _shot_state = SHOT;
            _dither_dist = dither_dist;
            _dither_to_shots = _dither_shots = dither_shots;
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
            if (!(--_shots)) {
                if (_dither_dist || _dither_shots) {
                    _dither_move = -_dither_cur_dist;
                    _dither_cur_dist = 0;
                    _do_dither = true;
                }
                stop_shutter();
            } else {
                if ((_dither_dist || _dither_shots) && !(--_dither_to_shots)) {
                    _dither_move = (int)random(0, _dither_dist + _dither_dist / 2);
                    if (_dither_cur_dist > 0)
                        _dither_move = -_dither_move;
                    _dither_move += -_dither_cur_dist;
                    _dither_cur_dist += _dither_move;
                    _do_dither = true;
                }
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

    void tick() {
        if (_do_dither) {
            _do_dither = false;
            move(llround(ra_to_asec(_dither_move) / SEC_PER_STEP));
            _dither_to_shots = _dither_shots;

            if (_shutter_pin != -1 && _shutter_timer) {
                timerWrite(_shutter_timer, _shot_intv);
                digitalWrite(_shutter_pin, 0);
            }
        }
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
    int _dither_dist = 0;
    int _dither_shots = 0;
    int _dither_to_shots = 0;
    int _dither_cur_dist = 0;
    int _dither_move = 0;
    bool _do_dither = false;
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
#ifdef WITH_DECMPU
void init(AstroTracker *tracker, DecMpu *mpu = nullptr) {
#else
void init(AstroTracker *tracker) {
#endif
        if (!_cats.init()) {
            Serial.println("FS not inited");
            Serial.println(F_STR(HR_CAT_NAME));
            Serial.println(F_STR(STARNAMES_HR_NAME));
        }
        _tracker = tracker;
#ifdef WITH_DECMPU
        if ((_mpu = mpu))
            mpu->init();
#endif

        // подключение к WiFi..
        Serial.print(F("MAC: "));
        Serial.println(WiFi.macAddress());

        WiFi.onEvent(on_wifi_sta_ip, ARDUINO_EVENT_WIFI_STA_GOT_IP);
        WiFi.onEvent(on_wifi_sta_dis, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
        WiFi.mode(WIFI_AP_STA);

# ifdef TRACKER_WIFI_AP
        WiFi.softAP(_ap_ssid, _ap_pass);
# else
        WiFi.begin(F_STR(STA_SSID), F_STR(STA_PASS));
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
        _tracker->tick();

        if (_hub.focused() && (us - _upd_tmr_us) >= _upd_intv_us) {
            _upd_tmr_us = us;
            if (_do_run)
                _runned_us = us;

#ifdef WITH_DECMPU
            _mpu->tick();
#endif

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
#ifdef WITH_DECMPU
            sec_to_comp(degrees(_mpu->ypr[1]) * 3600, d, m, s);
            snprintf(_sky_deg, sizeof(_sky_deg), "%3i° %2i' %6.03f\"", d, m, s);
            upd.update(F("sky_dec")).value(_sky_deg);
            upd.update(F("dec_y")).value(degrees(_mpu->ypr[0]));
            upd.update(F("dec_p")).value(degrees(_mpu->ypr[1]));
            upd.update(F("dec_r")).value(degrees(_mpu->ypr[2]));
#endif

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
#ifdef WITH_DECMPU
    DecMpu *_mpu = nullptr;
#endif
    Catalogues _cats;

    uint64_t _upd_intv_us = 1000000;
    uint64_t _upd_tmr_us = 0;
    uint64_t _run_time_us = 0;
    uint64_t _runned_us = 0;
    uint32_t _unix_ts = 0;
    bool _do_run = false;
    int _images = 0;
    int _dither_shots = CAMERA_DITHER;
    int _dither_dist = CAMERA_DITHER_DIST;
    int _exposure_s = CAMERA_EXPOSURE;
    int _intv_s = CAMERA_INTV;
    double _shot_delay_s = CAMERA_SHOT_DELAY;
    bool _do_exposure = false;
    char _sky_deg[32]{};
    char _move_rem_deg[32]{};

    // movement
    int _move_ra_h = 0, _move_ra_m = 0, _move_dec_d = 0, _move_dec_m = 0;
    double _move_ra_s = 0.0, _move_dec_s = 0.0;
    uint16_t _hr = 1;
    uint16_t _messier = 1;

    // network
    String _ap_ssid{F_STR(AP_SSID)},
        _ap_pass{F_STR(AP_PASS)},
        _sta_ssid{F_STR(STA_SSID)},
        _sta_pass{F_STR(STA_PASS)};

# ifdef TRACKER_WIFI_AP
    bool _ap_on = true, _sta_on = false;
# else
    bool _ap_on = false, _sta_on = true;
# endif

    void on_build(gh::Builder& b) {
        static byte tab;
        if (b.Tabs(&tab).text(F(GUI_TABS)).click())
            b.refresh();

        b.show(tab == MAIN_TAB);   // Main
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
        b.Label_(F("sky_deg")).value(0).label(F("deg")).disabled().size(7).fontSize(18);
        b.endRow();
        b.Label_(F("sky_dec")).value(0).label(F("DEC")).disabled().fontSize(18);

        b.Title(F("Movement")).fontSize(18);
        b.beginRow();
        b.Spinner_(F("move_ra_h"), &_move_ra_h).range(0, 23, 1).label(F("RA, h")).size(3);
        b.Spinner_(F("move_ra_m"), &_move_ra_m).range(0, 59, 1).label(F("RA, m")).size(3);
        b.Spinner_(F("move_ra_s"), &_move_ra_s).range(0.0, 60.0 - RA_SEC_PER_STEP, RA_SEC_PER_STEP, 3).label(F("RA, s")).size(4);
        b.endRow();
        b.beginRow();
        b.Spinner_(F("move_dec_d"), &_move_dec_d).range(-90, 90, 1).label(F("DEC, deg")).size(3).disabled();
        b.Spinner_(F("move_dec_m"), &_move_dec_m).range(-60, 60, 1).label(F("DEC, m")).size(3).disabled();
        b.Spinner_(F("move_dec_s"), &_move_dec_s).range(-60, 60, 1).label(F("DEC, s")).size(4).disabled();
        b.endRow();
        b.beginRow();
        b.Time_(F("move_rem_ra")).value(0).label(F("RA remaining")).disabled().size(3);
        b.Label_(F("move_rem_deg")).value(0).label(F("RA remaining (deg)")).disabled().size(7).fontSize(18);
        b.endRow();
        b.beginRow();
        b.Button_(F("move_bwd")).noLabel().icon(F("f2ea")).size(3).attach([this]() {
            this->_tracker->move(-llround(ra_to_asec(racomp_to_rasec(this->_move_ra_h, this->_move_ra_m, this->_move_ra_s)) / SEC_PER_STEP));
        });
        b.Button_(F("move_stop")).noLabel().icon(F("f04d")).size(4).attach([this]() {
            this->_tracker->stop_move();
        });
        b.Button_(F("move_fwd")).noLabel().icon(F("f2f9")).size(3).attach([this]() {
            this->_tracker->move(llround(ra_to_asec(racomp_to_rasec(this->_move_ra_h, this->_move_ra_m, this->_move_ra_s)) / SEC_PER_STEP));
        });
        b.endRow();
        b.beginRow();
        b.Button().label(F("move to")).icon(F("f78c")).size(1).attach([this]() {
        this->_tracker->move_to(llround(ra_to_asec(racomp_to_rasec(this->_move_ra_h, this->_move_ra_m, this->_move_ra_s)) / SEC_PER_STEP));
        });
        b.Button().label(F("set current")).icon(F("f111")).size(1).attach([this]() {
            this->_tracker->position_steps = llround(ra_to_asec(racomp_to_rasec(this->_move_ra_h, this->_move_ra_m, this->_move_ra_s)) / SEC_PER_STEP);
        });
        b.endRow();
        b.Title(F("Catalogues")).fontSize(18);
        b.beginRow();
        if (b.Spinner_(F("hr_num"), &_hr).label(F("HR number (1 - 9110)")).range(1, 9110, 1).size(3).click()) {
            cat_t entry;
            _cats.by_hr(_hr, _unix_ts, entry);
            ra_dec_set(entry);
        }
        if (b.Select().label(F("Star names")).text(F_STR(STARNAMES)).size(7).click()) {
            cat_t entry;
            _hr = _cats.by_satname_idx((uint16_t)b.build.value, _unix_ts, entry);
            ra_dec_set(entry);
            _hub.update(F("hr_num")).value(_hr);
        }
        b.endRow();
        b.beginRow();
        if (b.Spinner_(F("m_num"), &_messier).label(F("Messier (1 - 110)")).range(1, 110, 1).size(3).click()) {
            cat_t entry;
            _cats.by_messier(_messier, _unix_ts, entry);
            ra_dec_set(entry);
        }
        if (b.Select().label(F("Messier names")).text(F_STR(MESSIER_NAMES)).size(7).click()) {
            cat_t entry;
            _messier = _cats.by_m_name_idx((uint16_t)b.build.value, _unix_ts, entry);
            ra_dec_set(entry);
            _hub.update(F("m_num")).value(_messier);
        }
        b.endRow();
        b.beginRow();
        b.beginCol(3);
        if (b.Spinner_(F("ngc_num")).label(F("NGC 2000 (1 - 7840)")).range(1, NGC_MAX, 1).size(1).click()) {
            cat_t entry;
            _cats.by_ngc((uint16_t)b.build.value, _unix_ts, entry);
            ra_dec_set(entry);
        }
        if (b.Spinner_(F("ic_num")).label(F("IC (1 - 5386)")).range(1, IC_MAX, 1).size(1).click()) {
            cat_t entry;
            _cats.by_ic((uint16_t)b.build.value, _unix_ts, entry);
            ra_dec_set(entry);
        }
        b.endCol();
        if (b.Select().label(F("NGC/IC names")).text(F_STR(NGCIC_NAMES)).size(7).click()) {
            cat_t entry;
            int16_t ngcic = _cats.by_ngcic_name_idx((uint16_t)b.build.value, _unix_ts, entry);
            ra_dec_set(entry);
            _hub.update((ngcic > 0)? F("ngc_num"): F("ic_num")).value(abs(ngcic));
        }
        b.endRow();

        b.show(tab == CAPTURE_TAB);   // Capture
        b.beginRow();
        b.Time_(F("capt_exp"), &_exposure_s).label(F("Exposure time")).disabled(_do_exposure);
        b.Time_(F("capt_intv"), &_intv_s).label(F("Time between capture")).disabled(_do_exposure);
        b.endRow();
        b.beginRow();
        b.Spinner_(F("capt_delay"), &_shot_delay_s).label(F("Shot delay, s")).range(0.0, 1.0, 0.1).disabled(_do_exposure);
        b.Spinner_(F("capt_images"), &_images).label(F("Images to capture")).range(-1, INT_MAX, 1).disabled(_do_exposure).hint(F("-1 - infinite* mode"));
        b.endRow();
        b.beginRow();
        b.Spinner_(F("capt_dither_dist"), &_dither_dist).label(F("Dither distance, s")).range(0, 100, 1).disabled(_do_exposure);
        b.Spinner_(F("capt_dither"), &_dither_shots).label(F("Shots between dither")).range(0, 100, 1).disabled(_do_exposure);
        b.endRow();
        if (b.Confirm_(F("capt_conf")).text(F(_do_exposure? "Stop shuttering?": "Start shuttering?")).click()) {
            int x = b.build.value;
            Serial.print(F("Exposure "));
            if (!_do_exposure && x && _images && _intv_s >= 0 && _exposure_s > 0) {
                Serial.println(F("Begin!"));
                if ((_intv_s < CAMERA_MIN_INTV))
                    _intv_s = CAMERA_MIN_INTV;
                _tracker->start_shutter(_images, _intv_s, _exposure_s, _shot_delay_s, _dither_dist, _dither_shots);
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

#ifdef WITH_DECMPU
        b.show(tab == DEC_TAB);   // DEC control
        b.beginRow();
        b.Label_(F("dec_y")).label(F("yaw")).value(_mpu->ypr[0]);
        b.Label_(F("dec_p")).label(F("pitch")).value(_mpu->ypr[1]);
        b.Label_(F("dec_r")).label(F("roll")).value(_mpu->ypr[2]);
        b.endRow();
        b.beginRow();
        if (b.Confirm_(F("dec_calib")).text(F("Are you sure you want to start calibration?\n"
                                              "This may take a long time.\nMake sure tracking "
                                              "is turned off before starting.")).click()) {
            if (b.build.value) {
                _mpu->set_base(true);
                _mpu->calibrate();
            }
        }
        b.Button().label(F("Calibrate")).icon(F("f05b")).attach([this]() { this->_hub.sendAction(F("dec_calib")); });
        b.Button().label(F("Set base")).icon(F("f015")).attach([this]() { _mpu->set_base(); });
        b.endRow();
#endif

        b.show(tab == NETWORK_TAB);   // Network
        b.beginRow();
        b.Title(F("AP"));
        if (b.Switch(&_ap_on).click()) {
            int m = (_ap_on? WIFI_AP: 0) | (_sta_on? WIFI_STA: 0);
            WiFi.mode((wifi_mode_t)m);
            if (_ap_on)
                WiFi.softAP(_ap_ssid, _ap_pass);
        }
        b.endRow();
        b.Input_(F("ap_ssid"), &_ap_ssid).label(F("SSID")).value(F_STR(AP_SSID));
        b.Pass_(F("ap_pass"), &_ap_pass).value(F_STR(AP_PASS));
        if (b.Button().label(F("RUN")).click())
            WiFi.softAP(_ap_ssid, _ap_pass);

        b.beginRow();
        b.Title(F("STA"));
        if (b.Switch(&_sta_on).click()) {
            int m = (_ap_on? WIFI_AP: 0) | (_sta_on? WIFI_STA: 0);
            WiFi.mode((wifi_mode_t)m);
            if (_sta_on)
                WiFi.begin(_sta_ssid, _sta_pass);
        }
        b.endRow();
        b.Input_(F("sta_ssid"), &_sta_ssid).label(F("SSID"));//.value(F_STR(STA_SSID));
        b.Pass_(F("sta_pass"), &_sta_pass);//.value(F_STR(STA_PASS));
        if (b.Button().label(F("Connect")).click())
            WiFi.begin(_sta_ssid, _sta_pass);

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

    void ra_dec_set(cat_t &entry) {
        sec_to_comp(entry.ra, this->_move_ra_h, this->_move_ra_m, this->_move_ra_s);
        sec_to_comp(entry.dec, this->_move_dec_d, this->_move_dec_m, this->_move_dec_s, true);
        _hub.update(F("move_ra_h")).value(this->_move_ra_h);
        _hub.update(F("move_ra_m")).value(this->_move_ra_m);
        _hub.update(F("move_ra_s")).value(this->_move_ra_s);
        _hub.update(F("move_dec_d")).value(this->_move_dec_d);
        _hub.update(F("move_dec_m")).value(this->_move_dec_m);
        _hub.update(F("move_dec_s")).value(this->_move_dec_s);
    }

    void on_unix(uint32_t stamp) {
        _unix_ts = stamp;
    }
};


AstroTracker tracker(STEP_S_PIN, STEP_D_PIN, 1, SHUTTER_PIN, STEP_E_PIN);
#ifdef WITH_DECMPU
DecMpu mpu;
#endif
Interface iface;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    LittleFS.begin();
    Serial.println(F("setup"));
#ifdef WITH_DECMPU
    iface.init(&tracker, &mpu);
#else
    iface.init(&tracker);
#endif
    tracker.enable(true);
}

void loop() {
    iface.tick(esp_timer_get_time());
}
