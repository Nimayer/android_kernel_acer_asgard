
/*
 * Copyright (C) 2016 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef _MTK_EEM_INTERNAL_AP_H_
#define _MTK_EEM_INTERNAL_AP_H_

struct eem_det;
struct eem_ctrl;

struct eem_det_ops {

	/* interface to EEM */
	void (*enable)(struct eem_det *det, int reason);
	void (*disable)(struct eem_det *det, int reason);
	void (*disable_locked)(struct eem_det *det, int reason);
	void (*switch_bank)(struct eem_det *det, enum eem_phase phase);

	int (*init01)(struct eem_det *det);
	int (*init02)(struct eem_det *det);
	int (*mon_mode)(struct eem_det *det);

	int (*get_status)(struct eem_det *det);
	void (*dump_status)(struct eem_det *det);

	void (*set_phase)(struct eem_det *det, enum eem_phase phase);

	/* interface to thermal */
	int (*get_temp)(struct eem_det *det);

	/* interface to DVFS */
	int (*get_volt)(struct eem_det *det);
	int (*set_volt)(struct eem_det *det);
	void (*restore_default_volt)(struct eem_det *det);
	void (*get_freq_table)(struct eem_det *det);
	void (*get_orig_volt_table)(struct eem_det *det);

	/* interface to PMIC */
	int (*volt_2_pmic)(struct eem_det *det, int volt);
	int (*volt_2_eem)(struct eem_det *det, int volt);
	int (*pmic_2_volt)(struct eem_det *det, int pmic_val);
	int (*eem_2_pmic)(struct eem_det *det, int eev_val);
};

struct eem_det {
	const char *name;
	struct eem_det_ops *ops;
	int status;			/* TODO: enable/disable */
	int features;		/* enum eem_features */
	enum eem_ctrl_id ctrl_id;

	/* devinfo */
	unsigned int EEMINITEN;
	unsigned int EEMMONEN;
	unsigned int MDES;
	unsigned int BDES;
	unsigned int DCMDET;
	unsigned int DCBDET;
	unsigned int AGEDELTA;
	unsigned int MTDES;

	/* constant */
	unsigned int DETWINDOW;
	unsigned int VMAX;
	unsigned int VMIN;
	unsigned int DTHI;
	unsigned int DTLO;
	unsigned int VBOOT;
	unsigned int DETMAX;
	unsigned int AGECONFIG;
	unsigned int AGEM;
	unsigned int DVTFIXED;
	unsigned int VCO;
	unsigned int DCCONFIG;

	/* Generated by EEM init01. Used in EEM init02 */
	unsigned int DCVOFFSETIN;
	unsigned int AGEVOFFSETIN;

	/* for PMIC */
	unsigned int eem_v_base;
	unsigned int eem_step;
	unsigned int pmic_base;
	unsigned int pmic_step;

	/* for debug */
	unsigned int dcvalues[NR_EEM_PHASE];

	unsigned int freqpct30[NR_EEM_PHASE];
	unsigned int eem_26c[NR_EEM_PHASE];
	unsigned int vop30[NR_EEM_PHASE];
	unsigned int eem_eemEn[NR_EEM_PHASE];
	#if 0 /* no record table */
	u32 *recordRef;
	#endif
	#if DUMP_DATA_TO_DE
	unsigned int reg_dump_data[ARRAY_SIZE(reg_dump_addr_off)][NR_EEM_PHASE];
	#endif
	/* slope */
	unsigned int MTS;
	unsigned int BTS;
	unsigned int t250;
	/* dvfs */
	unsigned int num_freq_tbl; /* could be got @ the same time with freq_tbl[] */
	unsigned int max_freq_khz; /* maximum frequency used to calculate percentage */
	unsigned char freq_tbl[NR_FREQ]; /* percentage to maximum freq */

	unsigned int volt_tbl_orig[NR_FREQ]; /* orig volt table for restoreing to dvfs*/
	unsigned int volt_tbl[NR_FREQ]; /* pmic value */
	unsigned int volt_tbl_init2[NR_FREQ]; /* pmic value */
	unsigned int volt_tbl_pmic[NR_FREQ]; /* pmic value */
	unsigned int volt_tbl_bin[NR_FREQ]; /* pmic value */
	int volt_offset;
	int pi_offset;

	unsigned int disabled; /* Disabled by error or sysfs */
	unsigned char set_volt_to_upower; /* only when init2, eem need to set volt to upower */
};

struct eem_devinfo {
	/* M_HW_RES0 0x102A0580 */
	unsigned int BIG_BDES:8;
	unsigned int BIG_MDES:8;
	unsigned int BIG_DCBDET:8;
	unsigned int BIG_DCMDET:8;

	/* M_HW_RES1 0x102A0584 */
	unsigned int BIG_INITEN:1;
	unsigned int BIG_MONEN:1;
	unsigned int BIG_DVFS_LOW:2;
	unsigned int BIG_TURBO:1;
	unsigned int BIG_SPEC:3;
	unsigned int BIG_LEAKAGE:8;
	unsigned int BIG_MTDES:8;
	unsigned int BIG_AGEDELTA:8;

	/* M_HW_RES2 0x102A0588 */
	unsigned int CCI_BDES:8;
	unsigned int CCI_MDES:8;
	unsigned int CCI_DCBDET:8;
	unsigned int CCI_DCMDET:8;

	/* M_HW_RES3 0x102A058C */
	unsigned int CCI_INITEN:1;
	unsigned int CCI_MONEN:1;
	unsigned int CCI_DVFS_LOW:2;
	unsigned int CCI_TURBO:1;
	unsigned int CCI_SPEC:3;
	unsigned int CCI_LEAKAGE:8;
	unsigned int CCI_MTDES:8;
	unsigned int CCI_AGEDELTA:8;

	/* M_HW_RES4 0x102A0590 */
	unsigned int GPU_BDES:8;
	unsigned int GPU_MDES:8;
	unsigned int GPU_DCBDET:8;
	unsigned int GPU_DCMDET:8;

	/* M_HW_RES5 0x102A0594 */
	unsigned int GPU_INITEN:1;
	unsigned int GPU_MONEN:1;
	unsigned int GPU_DVFS_LOW:2;
	unsigned int GPU_TURBO:1;
	unsigned int GPU_SPEC:3;
	unsigned int GPU_LEAKAGE:8;
	unsigned int GPU_MTDES:8;
	unsigned int GPU_AGEDELTA:8;

	/* M_HW_RES6 0x102A0598 */
	unsigned int CPU_2L_BDES:8;
	unsigned int CPU_2L_MDES:8;
	unsigned int CPU_2L_DCBDET:8;
	unsigned int CPU_2L_DCMDET:8;

	/* M_HW_RES7 0x102A059C */
	unsigned int CPU_2L_INITEN:1;
	unsigned int CPU_2L_MONEN:1;
	unsigned int CPU_2L_DVFS_LOW:2;
	unsigned int CPU_2L_TURBO:1;
	unsigned int CPU_2L_SPEC:3;
	unsigned int CPU_2L_LEAKAGE:8;
	unsigned int CPU_2L_MTDES:8;
	unsigned int CPU_2L_AGEDELTA:8;

	/* M_HW_RES8 0x102A05A0 */
	unsigned int CPU_L_BDES:8;
	unsigned int CPU_L_MDES:8;
	unsigned int CPU_L_DCBDET:8;
	unsigned int CPU_L_DCMDET:8;

	/* M_HW_RES9 0x102A05A4 */
	unsigned int CPU_L_INITEN:1;
	unsigned int CPU_L_MONEN:1;
	unsigned int CPU_L_DVFS_LOW:2;
	unsigned int CPU_L_TURBO:1;
	unsigned int CPU_L_SPEC:3;
	unsigned int CPU_L_LEAKAGE:8;
	unsigned int CPU_L_MTDES:8;
	unsigned int CPU_L_AGEDELTA:8;

	/* M_HW_RES10 0x102A05B0 */
	unsigned int SOC_BDES:8;
	unsigned int SOC_MDES:8;
	unsigned int SOC_DCBDET:8;
	unsigned int SOC_DCMDET:8;

	/* M_HW_RES11 0x102A05B4 */
	unsigned int SOC_INITEN:1;
	unsigned int SOC_MONEN:1;
	unsigned int SOC_DVFS_LOW:2;
	unsigned int SOC_TURBO:1;
	unsigned int SOC_SPEC:3;
	unsigned int SOC_LEAKAGE:8;
	unsigned int SOC_MTDES:8;
	unsigned int SOC_AGEDELTA:8;

	/* M_HW_RES12 0X11F105E0 */
	unsigned int OD24_RESERVE:16;
	unsigned int OD24_PTP3:4;
	unsigned int FTPGM_VER:4;
	unsigned int OD24_RESERVE2:8;
};

/*********************************************
*extern variables defined at mtk_eem.c
*********************************************
*/
extern unsigned int freq[NR_FREQ];
extern unsigned int eem_vcore[VCORE_NR_FREQ];
extern unsigned int eem_vcore_index[VCORE_NR_FREQ];
extern unsigned char vcore_freq[NR_FREQ];
extern unsigned int vcore_opp[VCORE_NR_FREQ][4];

/* for setting pmic pwm mode and auto mode */
extern struct regulator *eem_regulator_proc1;
extern struct regulator *eem_regulator_proc2;
extern struct regulator *eem_regulator_gpu;
extern struct mutex record_mutex;
extern unsigned int record_tbl_locked[NR_FREQ]; /* table used to apply to dvfs at final */

/**************************************************
*extern variables and operations defined at mtk_eem_platform.c
***************************************************
*/
extern struct eem_det_ops big_det_ops;
extern struct eem_det_ops gpu_det_ops;
extern struct eem_det_ops soc_det_ops;
extern struct eem_det_ops little_det_ops;
extern struct eem_det_ops dual_little_det_ops;
extern struct eem_det_ops cci_det_ops;
extern struct eem_det_ops dmy_det_ops;

extern int get_volt_cpu(struct eem_det *det);
extern int set_volt_cpu(struct eem_det *det);
extern void restore_default_volt_cpu(struct eem_det *det);
extern void get_freq_table_cpu(struct eem_det *det);
extern void get_orig_volt_table_cpu(struct eem_det *det);
extern int get_volt_gpu(struct eem_det *det);
extern int set_volt_gpu(struct eem_det *det);
extern void restore_default_volt_gpu(struct eem_det *det);
extern void get_freq_table_gpu(struct eem_det *det);

#if EEM_BANK_SOC
extern void get_freq_table_vcore(struct eem_det *det);
extern int get_volt_vcore(struct eem_det *det);
extern int set_volt_vcore(struct eem_det *det);
extern void restore_volt_vcore(struct eem_det *det);
#endif

#if 0
int get_volt_lte(struct eem_det *det);
int set_volt_lte(struct eem_det *det);
void restore_default_volt_lte(struct eem_det *det);
#endif

/*********************************************
*extern operations defined at mtk_eem.c
*********************************************
*/
extern void base_ops_enable(struct eem_det *det, int reason);
extern void base_ops_disable(struct eem_det *det, int reason);
extern void base_ops_disable_locked(struct eem_det *det, int reason);
extern void base_ops_switch_bank(struct eem_det *det, enum eem_phase phase);

extern int base_ops_init01(struct eem_det *det);
extern int base_ops_init02(struct eem_det *det);
extern int base_ops_mon_mode(struct eem_det *det);

extern int base_ops_get_status(struct eem_det *det);
extern void base_ops_dump_status(struct eem_det *det);

extern void base_ops_set_phase(struct eem_det *det, enum eem_phase phase);
extern int base_ops_get_temp(struct eem_det *det);
extern int base_ops_get_volt(struct eem_det *det);
extern int base_ops_set_volt(struct eem_det *det);
extern void base_ops_restore_default_volt(struct eem_det *det);
extern void base_ops_get_freq_table(struct eem_det *det);
extern void base_ops_get_orig_volt_table(struct eem_det *det);
#endif

