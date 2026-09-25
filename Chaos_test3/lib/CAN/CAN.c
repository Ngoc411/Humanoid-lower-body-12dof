//#include "CAN.h"
//
//float test_angle_sp = 0.0f;
//
//// Filter
//void CAN_FilterConfig(CAN_HandleTypeDef *hcan) {
//    CAN_FilterTypeDef filter;
//    filter.FilterBank = 0;
//    filter.FilterMode = CAN_FILTERMODE_IDMASK;
//    filter.FilterScale = CAN_FILTERSCALE_32BIT;
//    filter.FilterIdHigh = 0x0000;
//    filter.FilterIdLow = 0x0000;
//    filter.FilterMaskIdHigh = 0x0000;
//    filter.FilterMaskIdLow = 0x0000;
//    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
//    filter.FilterActivation = CAN_FILTER_ENABLE;
//    filter.SlaveStartFilterBank = 14;
//
//    HAL_CAN_ConfigFilter(hcan, &filter);
//}
//
//// Transmit
//void CAN_Send(CAN_HandleTypeDef *hcan, uint32_t id, uint8_t *data, uint8_t len) {
//    CAN_TxHeaderTypeDef TxHeader;
//    uint32_t TxMailbox;
//
//    TxHeader.DLC = len;
//    TxHeader.StdId = id;
//    TxHeader.IDE = CAN_ID_STD;
//    TxHeader.RTR = CAN_RTR_DATA;
//
//    if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0) {
//        if (HAL_CAN_AddTxMessage(hcan, &TxHeader, data, &TxMailbox) != HAL_OK) {
//            // Error handling
//        }
//    }
//}
//
//void CAN_init(CAN_HandleTypeDef *hcan) {
//    CAN_FilterConfig(hcan);
//	HAL_CAN_Start(hcan);
//	HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
//}
//
///******************************************************************************/
//
//// CAN RX FIFO 0 Callback
//void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){
//    CAN_RxHeaderTypeDef RxHeader;
//    uint8_t RxData[8];
//
//    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
//        // if (RxData[0] == 0xAA && RxData[1] == 0x55) {
//        //     LED_GPIO_Port->BSRR = LED_Pin<<16;
//        // }
//        // else if (RxData[0] == 0x55 && RxData[1] == 0xAA) {
//        //     LED_GPIO_Port->BSRR = LED_Pin;
//        // }
//        if (RxData[0] == 0x30) {
//            union { float f; uint8_t b[4]; } u;
//            u.b[0] = RxData[1];
//            u.b[1] = RxData[2];
//            u.b[2] = RxData[3];
//            u.b[3] = RxData[4];
//
//            test_angle_sp = u.f;
//        }
//    }
//}
//
///******************************************************************************/
