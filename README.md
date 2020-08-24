# stm32g431_optiobyte_ll

1. PVD Level 6으로 2.9V 로 설정

   - 총 6 Level

      *  VPVD0 around 2.0 V

      *  VPVD1 around 2.2 V

      *  VPVD2 around 2.4 V

      *  VPVD3 around 2.5 V

      *  VPVD4 around 2.6 V

      *  VPVD5 around 2.8 V

      *  VPVD6 around 2.9 V

   - VDD가 해당 threshold 보다 낮아지거나 높아질 때 Interrupt가 발생

   
2. BOR은 Level 2로 2.31 V 로 설정하였습니다.

   
   - 총 5개의 Level이 있습니다.

      * VBOR0 : 1.66 V

      * VBOR1 : 2.10 V

      * VBOR2 : 2.31 V

      * VBOR3 : 2.61 V

      * VBOR4 : 2.90 V

   - BOR은 Option Byte Flash memody에 기록하는 방법 구현.

   - 최초 한번 Writing을 하면 다시 할 필요가 없음.

 
