TCD1304_stm32f401re_HAL : SPL library -> stm32cubeIDE 환경 사용 가능하도록 변환 작업, HAL library 사용

orginal code and hardware setting : using SPL, stm32f401re nucleo board, tcd1304
https://tcd1304.wordpress.com/ 참고해 동일하게 세팅

What's different from that code?
1. using stm32cubeIDE and HAL library
2. TIM2 ch2 -> TIM1 ch2 (SH) : 작동이 안해서 오실로스코프로 보니 tim2가 제대로 작동이 되지않아 tim1을 사용해보니 작동했다.

위 페이지의 pyccdgui 프로그램으로 동작을 확인했는데 single mode은 잘 동작하나 continuous mode는 동작은 하는데 값이 좀 튀는 현상이 일어난다.

