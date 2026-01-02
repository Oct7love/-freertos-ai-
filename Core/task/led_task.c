 #include "led_task.h"

  // ==================== IPC对象 ====================
  static EventGroupHandle_t led_event_group = NULL;
  static TimerHandle_t led_timer = NULL;

  // ==================== LED模式 ====================
  typedef enum {
      LED_MODE_HEARTBEAT = 0,
      LED_MODE_ERROR,
      LED_MODE_BUSY
  } led_mode_t;

  static led_mode_t current_mode = LED_MODE_HEARTBEAT;
  static uint8_t led_state = 0;

  // ==================== 定时器回调 ====================
  static void led_timer_callback(TimerHandle_t xTimer) {
      (void)xTimer;

      // BUSY模式下不翻转
      if (current_mode == LED_MODE_BUSY) {
          HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
          return;
      }

      // 翻转LED
      led_state = !led_state;
      HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin,
                      led_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }

  // ==================== LED任务 ====================
  static void led_control_task(void *arg) {
      EventBits_t events;

      // 延迟50ms，避免启动时打印冲突
      vTaskDelay(pdMS_TO_TICKS(50));
      uart_printf_dma(&huart1, "[LED Task] Running!\r\n");

      // 启动定时器（使用0超时，避免死锁）
      if (xTimerStart(led_timer, 0) == pdPASS) {
          uart_printf_dma(&huart1, "[LED] Timer started OK!\r\n");
      } else {
          uart_printf_dma(&huart1, "[ERROR] Timer start failed!\r\n");
          for(;;);  // 定时器启动失败，停止运行
      }

      for (;;) {
          // 完全阻塞等待事件
          events = xEventGroupWaitBits(
              led_event_group,
              LED_EVENT_HEARTBEAT | LED_EVENT_ERROR | LED_EVENT_BUSY,
              pdTRUE,
              pdFALSE,
              portMAX_DELAY
          );

          // 模式切换
          if (events & LED_EVENT_ERROR) {
              current_mode = LED_MODE_ERROR;
              xTimerChangePeriod(led_timer, pdMS_TO_TICKS(100), 0);
              uart_printf_dma(&huart1, "[LED] Mode -> ERROR (100ms)\r\n");

          } else if (events & LED_EVENT_BUSY) {
              current_mode = LED_MODE_BUSY;
              xTimerStop(led_timer, 0);
              HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
              uart_printf_dma(&huart1, "[LED] Mode -> BUSY (ON)\r\n");

          } else if (events & LED_EVENT_HEARTBEAT) {
              current_mode = LED_MODE_HEARTBEAT;
              xTimerChangePeriod(led_timer, pdMS_TO_TICKS(500), 0);
              uart_printf_dma(&huart1, "[LED] Mode -> HEARTBEAT (500ms)\r\n");
          }
      }
  }

  // ==================== 外部接口 ====================
  void led_set_mode(uint32_t event) {
      if (led_event_group) {
          xEventGroupSetBits(led_event_group, event);
      }
  }

  void led_task_init(void) {
      // 创建事件组
      led_event_group = xEventGroupCreate();
      if (!led_event_group) {
          uart_printf_dma(&huart1, "[ERROR] LED event group failed!\r\n");
          for(;;);
      }

      // 创建软件定时器（初始周期500ms，自动重载）
      led_timer = xTimerCreate(
          "led_timer",
          pdMS_TO_TICKS(500),
          pdTRUE,  // 自动重载
          NULL,
          led_timer_callback
      );
      if (!led_timer) {
          uart_printf_dma(&huart1, "[ERROR] LED timer failed!\r\n");
          for(;;);
      }

      // 创建LED控制任务
      if (xTaskCreate(led_control_task, "led_ctrl", 128, NULL, 2, NULL) != pdPASS) {
          uart_printf_dma(&huart1, "[ERROR] LED task failed!\r\n");
          for(;;);
      }

      uart_printf_dma(&huart1, "[LED] Init OK! (Pure event-driven)\r\n");
  }