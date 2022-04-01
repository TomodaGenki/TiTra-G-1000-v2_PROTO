/*
 * can.c
 *
 *  Created on: 2021/11/10
 *      Author: Katte
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "stm32f4xx_hal_can.h"
#include "common_func.h"
#include "wheel.h"

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
/* Private define ------------------------------------------------------------*/
#define RING_LENGTH	100
#define BUF_LENGTH	10
#define DATA_LENGTH	8

/* Private variables ---------------------------------------------------------*/
static uint8_t can1_first = 0;	// �����O�o�b�t�@�̎��o���ʒu
static uint8_t can1_last = 0;	// �����O�o�b�t�@�̊i�[�ʒu
static uint8_t can1_ring_buf[RING_LENGTH][BUF_LENGTH];
// �����O�o�b�t�@�̃f�[�^�\��
// 1�����ځG�����O�o�b�t�@��index
// 2�����ځF[id_low][id_high][d0][d1][d2][d3][d4][d5][d6][d7]

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
/*
 * CAN���b�Z�[�W��M���ɃR�[���o�b�N�����֐�
 * ��M�f�[�^��ID���ƂɐU�蕪���ăX�g�A����
 */
	CAN_RxHeaderTypeDef   RxHeader;
	uint8_t	Rxdata_tmp[8] = {0,0,0,0,0,0,0,0};
	if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, Rxdata_tmp) == HAL_OK){
		switch(RxHeader.StdId){
//		�o�b�e���[�̏����@���������邽�߂Ɋ撣��Ȃ���
//		case 0x100:
//			memcpy(RxData_id100, Rxdata_tmp, sizeof(Rxdata_tmp));
//			break;
//		case 0x101:
//			memcpy(RxData_id101, Rxdata_tmp, sizeof(Rxdata_tmp));
//			break;

		case (SDO_RX_ID + L_WHEEL_ID):
			// �����s���[�^�[�h���C�o����̕ԓ�
			receive_wheel_motor_data(Rxdata_tmp, L_WHEEL_ID);
			break;

		case (SDO_RX_ID + R_WHEEL_ID):
			// �E���s���[�^�[�h���C�o����̕ԓ�
			receive_wheel_motor_data(Rxdata_tmp, R_WHEEL_ID);
			break;

		case (EMCY_ID + L_WHEEL_ID):
			// �����s���[�^�[�h���C�o����̃G�}�[�W�F���V�[���b�Z�[�W
			receive_wheel_motor_error_data(Rxdata_tmp, L_WHEEL_ID);
			break;

		case (EMCY_ID + R_WHEEL_ID):
			// �E���s���[�^�[�h���C�o����̃G�}�[�W�F���V�[���b�Z�[�W
			receive_wheel_motor_error_data(Rxdata_tmp, R_WHEEL_ID);
			break;

		case 0x701:
			// ���[�^�[�h���C�o����̃n�[�g�r�[�g
			break;
		}
	}
}

// CAN��M�t�B���^�̐ݒ�
void can_filter_setting(void) {

	// �SID����M����
	CAN_FilterTypeDef filter;
	filter.FilterIdHigh         = 0;                        // �t�B���^�[ID(���16�r�b�g)
	filter.FilterIdLow          = 0;                        // �t�B���^�[ID(����16�r�b�g)
	filter.FilterMaskIdHigh     = 0;                        // �t�B���^�[�}�X�N(���16�r�b�g)
	filter.FilterMaskIdLow      = 0;                        // �t�B���^�[�}�X�N(����16�r�b�g)
	filter.FilterScale          = CAN_FILTERSCALE_32BIT;    // �t�B���^�[�X�P�[��
	filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;         // �t�B���^�[�Ɋ��蓖�Ă�FIFO
	filter.FilterBank           = 0;                        // �t�B���^�[�o���NNo
	filter.FilterMode           = CAN_FILTERMODE_IDMASK;    // �t�B���^�[���[�h
	filter.SlaveStartFilterBank = 14;                       // �X���[�uCAN�̊J�n�t�B���^�[�o���NNo
	filter.FilterActivation     = ENABLE;                   // �t�B���^�[�����^�L��
	HAL_CAN_ConfigFilter(&hcan1, &filter);
}

// CAN���M�p�̊֐�
HAL_StatusTypeDef can1_transmit(void) {

	static CAN_TxHeaderTypeDef TxHeader;
	uint32_t TxMailbox;
	uint32_t remain_box = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
	uint16_t id;
	uint8_t data[8];
	HAL_StatusTypeDef result = HAL_OK;

	// ���M����f�[�^�������Ȃ邩���[���{�b�N�X�̋󂫂������Ȃ�܂ő��M���s��
//	while (remain_box > 0) {
	if (remain_box > 0) {
		// CAN�̑��MBOX�ɋ󂫂�����ꍇ�̂ݑ��M���s��
		if (can1_deque(&id, data)) {
			// CAN�̐ݒ�
			TxHeader.StdId = id;			// CAN ID
			TxHeader.RTR = CAN_RTR_DATA;	// �f�[�^�t���[�����w��
			TxHeader.IDE = CAN_ID_STD;		// 11 bit ID (�W��ID)
			TxHeader.DLC = 8;				// Data Length 8Byte
			TxHeader.TransmitGlobalTime = DISABLE;

			result = HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, &TxMailbox);

			// �ēx���[���{�b�N�X�̋󂫂𒲂ׂ�
			remain_box = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);
		} else {
			// ���M����f�[�^��������΃��[�v�𔲂���
//			break;
		}
	}
	return result;
}

uint8_t can1_enque(uint16_t id, uint8_t *pdata) {
	// �����O�o�b�t�@�ɒl���i�[���邽�߂̊֐�
	union LongByte id_cnv;
	uint8_t ret = 0;

	// �����O�o�b�t�@�ɗ]�T�����邩�ǂ������m�F
	// �i�[�ʒu�̎����擪�ʒu�ł���Ύ��s
	if ((can1_last + 1) != can1_first) {
		id_cnv.l_val = 0;	//���p�̂̏�����
		id_cnv.w_val[0] = id;

		can1_ring_buf[can1_last][0] = id_cnv.b_val[0];
		can1_ring_buf[can1_last][1] = id_cnv.b_val[1];

		for (int i = 0; i < DATA_LENGTH; i++) {
			can1_ring_buf[can1_last][i+2] = pdata[i];
		}

		// �����O�o�b�t�@�̊i�[�ʒu���X�V
		can1_last++;
		if (can1_last >= RING_LENGTH) {
			can1_last = 0;
		}
		ret = 1;
	}

	return ret;
}

uint8_t can1_deque(uint16_t *p_id, uint8_t *pdata) {
	// �L���[����f�[�^�����o�����߂̊֐�
	union LongByte id_cnv;
	uint8_t ret = 0;

	// ���M���ׂ��f�[�^�����邩�ǂ������m�F
	// �擪�ʒu�Ɗi�[�ʒu������ł���΁A���M����f�[�^�����Ɣ��f
	if (can1_first != can1_last) {
		id_cnv.l_val = 0;	//���p�̂̏�����
		id_cnv.b_val[0] = can1_ring_buf[can1_first][0];
		id_cnv.b_val[1] = can1_ring_buf[can1_first][1];
		*p_id = id_cnv.w_val[0];

		for (int i = 0; i < DATA_LENGTH; i++) {
			pdata[i] = can1_ring_buf[can1_first][i+2];
		}
		// �����O�o�b�t�@�̐擪�ʒu���X�V
		can1_first++;
		if (can1_first >= RING_LENGTH) {
			can1_first = 0;
		}
		ret = 1;
	}

	return ret;
}
