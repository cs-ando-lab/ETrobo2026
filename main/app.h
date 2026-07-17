#ifdef __cplusplus
extern "C" {
#endif

#include <kernel.h>

/* タスク優先度 */
#define MAIN_PRIORITY 5      /* メインタスク */
#define DEBUG_LOG_PRIORITY 6 /* センサー値のBLE Monitor送信タスク（周期起動） */

/* タスク周期の定義 */
#define DEBUG_LOG_PERIOD (100 * 1000) /* センサー値送信タスク: 100msec周期 */

#ifndef STACK_SIZE
#define STACK_SIZE (4096)
#endif /* STACK_SIZE */

#ifndef TOPPERS_MACRO_ONLY

extern void main_task(intptr_t exinf);
extern void debug_log_task(intptr_t exinf);

#endif /* TOPPERS_MACRO_ONLY */

#ifdef __cplusplus
}
#endif