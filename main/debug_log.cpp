#include "debug_log.h"

#if DEBUG_LOG

#include <t_syslog.h>
#include "IMU.h"
#include "Button.h"

using namespace spikeapi;

/*
 * TOPPERSのsyslog()は1回の呼び出しにつき最大5個までの引数(TNUM_LOGPAR=6、
 * うち1個はフォーマット文字列自身が占有)しか処理できない。
 * そのため、項目数の多いセンサーは「count + データ最大4個」になるよう
 * 複数のタグ(COLOR/COLOR2/...)に分割して送信する。
 */

void debug_log_init(const debug_sensors_t* sensors) {
    if(sensors->color)
        sensors->color->lightOn();
    IMU imu; /* hub_imu_init() を実行する */
}

void debug_log_all(const debug_sensors_t* sensors, int count) {
    /* 値が変化した時だけ送信（デルタ送信） */
    switch(count % 9) {
        case 0: {
            if(!sensors->color)
                break;
            ColorSensor::RGB rgb;
            sensors->color->getRGB(rgb);
            int32_t refl = sensors->color->getReflection();
            static int32_t pr = -1, pg = -1, pb = -1, prefl = -1;
            if(rgb.r != pr || rgb.g != pg || rgb.b != pb || refl != prefl) {
                syslog(LOG_NOTICE, "COLOR,%d,%d,%d,%d,%d", count, rgb.r, rgb.g, rgb.b, refl);
                pr = rgb.r;
                pg = rgb.g;
                pb = rgb.b;
                prefl = refl;
            }
            break;
        }
        case 1: {
            if(!sensors->color)
                break;
            /* getColor()は6色に丸め込んだ基準値を返すため、ColorJudgeの調整には
             * 丸め込み前の生の値が見えるgetHSV()を使う */
            ColorSensor::HSV rawColor;
            sensors->color->getHSV(rawColor, true);
            static int32_t pah = -1;
            static int pas = -1, pav = -1;
            if(rawColor.h != pah || rawColor.s != pas || rawColor.v != pav) {
                syslog(LOG_NOTICE, "COLOR2,%d,%d,%d,%d", count, rawColor.h, rawColor.s, rawColor.v);
                pah = rawColor.h;
                pas = rawColor.s;
                pav = rawColor.v;
            }
            break;
        }
        case 2: {
            if(!sensors->left_motor || !sensors->right_motor)
                break;
            int32_t lspd = sensors->left_motor->getSpeed();
            int32_t rspd = sensors->right_motor->getSpeed();
            int32_t lcnt = sensors->left_motor->getCount();
            int32_t rcnt = sensors->right_motor->getCount();
            static int32_t pls = -9999, prs = -9999, plc = -9999, prc = -9999;
            if(lspd != pls || rspd != prs || lcnt != plc || rcnt != prc) {
                syslog(LOG_NOTICE, "MOTOR,%d,%d,%d,%d,%d", count, lspd, rspd, lcnt, rcnt);
                pls = lspd;
                prs = rspd;
                plc = lcnt;
                prc = rcnt;
            }
            break;
        }
        case 3: {
            if(!sensors->left_motor || !sensors->right_motor)
                break;
            int32_t lpow = sensors->left_motor->getPower();
            int32_t rpow = sensors->right_motor->getPower();
            int lstall = sensors->left_motor->isStalled() ? 1 : 0;
            int rstall = sensors->right_motor->isStalled() ? 1 : 0;
            static int32_t plp = -9999, prp = -9999;
            static int plst = -1, prst = -1;
            if(lpow != plp || rpow != prp || lstall != plst || rstall != prst) {
                syslog(LOG_NOTICE, "MOTOR2,%d,%d,%d,%d,%d", count, lpow, rpow, lstall, rstall);
                plp = lpow;
                prp = rpow;
                plst = lstall;
                prst = rstall;
            }
            break;
        }
        case 4: {
            static IMU imu;
            IMU::Acceleration acc;
            imu.getAcceleration(acc);
            int heading = (int)imu.getHeading();
            int ax = (int)acc.x, ay = (int)acc.y, az = (int)acc.z;
            static int ph = -9999, pax = -9999, pay = -9999, paz = -9999;
            if(heading != ph || ax != pax || ay != pay || az != paz) {
                syslog(LOG_NOTICE, "IMU,%d,%d,%d,%d,%d", count, heading, ax, ay, az);
                ph = heading;
                pax = ax;
                pay = ay;
                paz = az;
            }
            break;
        }
        case 5: {
            static IMU imu;
            IMU::AngularVelocity angv;
            imu.getAngularVelocity(angv);
            int avx = (int)angv.x, avy = (int)angv.y, avz = (int)angv.z;
            static int pavx = -9999, pavy = -9999, pavz = -9999;
            if(avx != pavx || avy != pavy || avz != pavz) {
                syslog(LOG_NOTICE, "IMU2,%d,%d,%d,%d", count, avx, avy, avz);
                pavx = avx;
                pavy = avy;
                pavz = avz;
            }
            break;
        }
        case 6: {
            if(!sensors->ultrasonic)
                break;
            int32_t dist = sensors->ultrasonic->getDistance();
            static int32_t pd = -9999;
            if(dist != pd) {
                syslog(LOG_NOTICE, "ULTRA,%d,%d", count, dist);
                pd = dist;
            }
            break;
        }
        case 7: {
            if(!sensors->force)
                break;
            int touched = sensors->force->isTouched() ? 1 : 0;
            int force_x100 = (int)(sensors->force->getForce() * 100.0f);
            int dist_x100 = (int)(sensors->force->getDistance() * 100.0f);
            static int pt = -1, pf = -9999, pdst = -9999;
            if(touched != pt || force_x100 != pf || dist_x100 != pdst) {
                syslog(LOG_NOTICE, "FORCE,%d,%d,%d,%d", count, touched, force_x100, dist_x100);
                pt = touched;
                pf = force_x100;
                pdst = dist_x100;
            }
            break;
        }
        case 8: {
            static Button button;
            int left = button.isLeftPressed() ? 1 : 0;
            int center = button.isCenterPressed() ? 1 : 0;
            int right = button.isRightPressed() ? 1 : 0;
            int bt = button.isBluetoothPressed() ? 1 : 0;
            static int pl = -1, pc2 = -1, pr2 = -1, pbt = -1;
            if(left != pl || center != pc2 || right != pr2 || bt != pbt) {
                syslog(LOG_NOTICE, "BUTTON,%d,%d,%d,%d,%d", count, left, center, right, bt);
                pl = left;
                pc2 = center;
                pr2 = right;
                pbt = bt;
            }
            break;
        }
    }
}

#endif /* DEBUG_LOG */
