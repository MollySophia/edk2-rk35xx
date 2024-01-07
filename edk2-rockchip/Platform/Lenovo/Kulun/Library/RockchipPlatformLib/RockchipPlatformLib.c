/** @file
*
*  Copyright (c) 2021, Rockchip Limited. All rights reserved.
*
*  SPDX-License-Identifier: BSD-2-Clause-Patent
*
**/
#include <Base.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/GpioLib.h>
#include <Library/RK806.h>
#include <Library/Rk3588Pcie.h>
#include <Library/PWMLib.h>
#include <Soc.h>

static struct regulator_init_data rk806_init_data[] = {
  /* Master PMIC */
  RK8XX_VOLTAGE_INIT(MASTER_BUCK1, 750000), // vdd_gpu_s0 550000~950000
  RK8XX_VOLTAGE_INIT(MASTER_BUCK3, 750000), // vdd_log_s0
  RK8XX_VOLTAGE_INIT(MASTER_BUCK4, 750000), // vdd_vdenc_s0 550000~950000
  RK8XX_VOLTAGE_INIT(MASTER_BUCK5, 850000), // vdd_gpu_mem_s0 650000~950000
  RK8XX_VOLTAGE_INIT(MASTER_BUCK7, 2000000), // vdd_2v0_pldo_s3
  RK8XX_VOLTAGE_INIT(MASTER_BUCK8, 750000), // vdd_vdenc_mem_s0 675000~950000
  RK8XX_VOLTAGE_INIT(MASTER_BUCK10, 1100000), // vcc_1v1_nldo_s3

  RK8XX_VOLTAGE_INIT(MASTER_NLDO1, 750000), // vdd_0v75_s3
  RK8XX_VOLTAGE_INIT(MASTER_NLDO2, 900000), // vdd2l_0v9_ddr_s3
  RK8XX_VOLTAGE_INIT(MASTER_NLDO3, 750000), // vdd_0v75_hdmi_edp_s0
  RK8XX_VOLTAGE_INIT(MASTER_NLDO4, 750000), // avdd_0v75_s0
  RK8XX_VOLTAGE_INIT(MASTER_NLDO5, 850000), // vdd_0x85_s0

  RK8XX_VOLTAGE_INIT(MASTER_PLDO1, 1800000), // avcc_1v8_s0
  RK8XX_VOLTAGE_INIT(MASTER_PLDO2, 1800000), // vdd1_1v8_ddr_s3
  RK8XX_VOLTAGE_INIT(MASTER_PLDO3, 1800000), // avcc_1v8_codec_s0
  RK8XX_VOLTAGE_INIT(MASTER_PLDO4, 3300000), // vcc_3v3_s3
  // RK8XX_VOLTAGE_INIT(MASTER_PLDO5, 3300000), //vccio_sd_s0
  RK8XX_VOLTAGE_INIT(MASTER_PLDO6, 1800000), // vccio_1v8_s3

  // Slave RK806

  RK8XX_VOLTAGE_INIT(SLAVER_BUCK1, 750000), // vdd_cpu_big1_s0 550000~1050000
  RK8XX_VOLTAGE_INIT(SLAVER_BUCK2, 750000), // vdd_cpu_big0_s0 550000~1050000
  RK8XX_VOLTAGE_INIT(SLAVER_BUCK3, 750000), // vdd_cpu_lit_s0 550000~950000
  RK8XX_VOLTAGE_INIT(SLAVER_BUCK4, 3300000), // vcc_3v3_s0
  RK8XX_VOLTAGE_INIT(SLAVER_BUCK5, 750000), // vdd_cpu_big1_mem_s0 675000~1050000
  RK8XX_VOLTAGE_INIT(SLAVER_BUCK6, 750000), // vdd_cpu_big0_mem_s0 675000~1050000
  RK8XX_VOLTAGE_INIT(SLAVER_BUCK7, 1800000), // vcc_1v8_s0
  RK8XX_VOLTAGE_INIT(SLAVER_BUCK8, 750000), // vdd_cpu_lit_mem_s0 675000~950000
  RK8XX_VOLTAGE_INIT(SLAVER_BUCK10, 850000), // vdd_ddr_s0

  RK8XX_VOLTAGE_INIT(SLAVER_NLDO1, 750000),
  RK8XX_VOLTAGE_INIT(SLAVER_NLDO2, 850000),
  RK8XX_VOLTAGE_INIT(SLAVER_NLDO3, 850000),
  RK8XX_VOLTAGE_INIT(SLAVER_NLDO4, 1200000),
  RK8XX_VOLTAGE_INIT(SLAVER_NLDO5, 1200000),

  RK8XX_VOLTAGE_INIT(SLAVER_PLDO1, 1800000),
  RK8XX_VOLTAGE_INIT(SLAVER_PLDO2, 1800000),
  RK8XX_VOLTAGE_INIT(SLAVER_PLDO3, 1800000),
  RK8XX_VOLTAGE_INIT(SLAVER_PLDO4, 3300000),
  RK8XX_VOLTAGE_INIT(SLAVER_PLDO5, 2800000),
  RK8XX_VOLTAGE_INIT(SLAVER_PLDO6, 1800000),
};

VOID
EFIAPI
SdmmcIoMux (
  VOID
  )
{

}

VOID
EFIAPI
SdhciEmmcIoMux (
  VOID
  )
{
  /* sdhci0 iomux (eMMC socket) */
  BUS_IOC->GPIO2A_IOMUX_SEL_L = (0xFFFFUL << 16) | (0x1111); //EMMC_CMD,EMMC_CLKOUT,EMMC_DATASTROBE,EMMC_RSTN
  BUS_IOC->GPIO2D_IOMUX_SEL_L = (0xFFFFUL << 16) | (0x1111); //EMMC_D0,EMMC_D1,EMMC_D2,EMMC_D3
  BUS_IOC->GPIO2D_IOMUX_SEL_H = (0xFFFFUL << 16) | (0x1111); //EMMC_D4,EMMC_D5,EMMC_D6,EMMC_D7
}

#define NS_CRU_BASE         0xFD7C0000
#define CRU_CLKSEL_CON59    0x03EC
#define CRU_CLKSEL_CON78    0x0438

VOID
EFIAPI
Rk806SpiIomux (
  VOID
  )
{
  /* io mux */
  //BUS_IOC->GPIO1A_IOMUX_SEL_H = (0xFFFFUL << 16) | 0x8888;
  //BUS_IOC->GPIO1B_IOMUX_SEL_L = (0x000FUL << 16) | 0x0008;
  PMU1_IOC->GPIO0A_IOMUX_SEL_H = (0x0FF0UL << 16) | 0x0110;
  PMU1_IOC->GPIO0B_IOMUX_SEL_L = (0xF0FFUL << 16) | 0x1011;
  MmioWrite32(NS_CRU_BASE + CRU_CLKSEL_CON59, (0x00C0UL << 16) | 0x0080);
}

VOID
EFIAPI
Rk806Configure (
  VOID
  )
{
  UINTN RegCfgIndex;

  RK806Init();

  for (RegCfgIndex = 0; RegCfgIndex < ARRAY_SIZE(rk806_init_data); RegCfgIndex++)
    RK806RegulatorInit(rk806_init_data[RegCfgIndex]);
}

VOID
EFIAPI
SetCPULittleVoltage (
  IN UINT32 Microvolts
  )
{
  struct regulator_init_data Rk806CpuLittleSupply =
        RK8XX_VOLTAGE_INIT(SLAVER_BUCK3, Microvolts);

  RK806RegulatorInit(Rk806CpuLittleSupply);
}

VOID
EFIAPI
SetCPUBigVoltage (
  IN UINT32 Microvolts
  )
{
  struct regulator_init_data Rk806CpuBig01Supply =
        RK8XX_VOLTAGE_INIT(SLAVER_BUCK2, Microvolts);

  struct regulator_init_data Rk809CpuBig23Supply = 
        RK8XX_VOLTAGE_INIT(SLAVER_BUCK1, Microvolts);

  RK806RegulatorInit(Rk806CpuBig01Supply);
  RK806RegulatorInit(Rk809CpuBig23Supply);
}


VOID
EFIAPI
NorFspiIomux (
  VOID
  )
{
  /* io mux */
  MmioWrite32(NS_CRU_BASE + CRU_CLKSEL_CON78,
             (((0x3 << 12) | (0x3f << 6)) << 16) | (0x0 << 12) | (0x3f << 6));
#define FSPI_M1
#if defined(FSPI_M0)
   /*FSPI M0*/
  BUS_IOC->GPIO2A_IOMUX_SEL_L = ((0xF << 0) << 16) | (2 << 0); //FSPI_CLK_M0
  BUS_IOC->GPIO2D_IOMUX_SEL_L = (0xFFFFUL << 16) | (0x2222); //FSPI_D0_M0,FSPI_D1_M0,FSPI_D2_M0,FSPI_D3_M0
  BUS_IOC->GPIO2D_IOMUX_SEL_H = ((0xF << 8) << 16) | (0x2 << 8); //FSPI_CS0N_M0
#elif defined(FSPI_M1)
  /*FSPI M1*/
  BUS_IOC->GPIO2A_IOMUX_SEL_H = (0xFF00UL << 16) | (0x3300); //FSPI_D0_M1,FSPI_D1_M1
  BUS_IOC->GPIO2B_IOMUX_SEL_L = (0xF0FFUL << 16) | (0x3033); //FSPI_D2_M1,FSPI_D3_M1,FSPI_CLK_M1
  BUS_IOC->GPIO2B_IOMUX_SEL_H = (0xF << 16) | (0x3); //FSPI_CS0N_M1
#else
  /*FSPI M2*/
  BUS_IOC->GPIO3A_IOMUX_SEL_L = (0xFFFFUL << 16) | (0x5555); //[FSPI_D0_M2-FSPI_D3_M2]
  BUS_IOC->GPIO3A_IOMUX_SEL_H = (0xF0UL << 16) | (0x50); //FSPI_CLK_M2
  BUS_IOC->GPIO3C_IOMUX_SEL_H = (0xF << 16) | (0x2); //FSPI_CS0_M2
#endif
}

VOID
EFIAPI
GmacIomux (
   UINT32 id
  )
{
  /* No GMAC here */
}

VOID
EFIAPI
NorFspiEnableClock (
  UINT32 *CruBase
  )
{
  UINTN BaseAddr = (UINTN) CruBase;

  MmioWrite32(BaseAddr + 0x087C, 0x0E000000);
}

VOID
EFIAPI
I2cIomux (
   UINT32 id
  )
{
  switch (id) {
  case 0:
    break;
  case 1:
    break;
  case 2:
    break;
  case 3:
    break;
  case 4:
    GpioPinSetFunction(1, GPIO_PIN_PA3, 9); //i2c4_scl_m3
    GpioPinSetFunction(1, GPIO_PIN_PA2, 9); //i2c4_sda_m3
    break;
  case 5:
    break;
  case 6:
    break;
  default:
    break;
  }
}

VOID
EFIAPI
UsbPortPowerEnable (
  VOID
  )
{
  DEBUG((DEBUG_INFO, "UsbPortPowerEnable called\n"));
  /* Set GPIO4 PB0 (USB_HOST_PWREN) output high to power USB ports */
  GpioPinWrite (4, GPIO_PIN_PB0, TRUE);
  GpioPinSetDirection (4, GPIO_PIN_PB0, GPIO_PIN_OUTPUT);

  GpioPinWrite (3, GPIO_PIN_PB1, TRUE);
  GpioPinSetDirection (3, GPIO_PIN_PB1, GPIO_PIN_OUTPUT);

  DEBUG((DEBUG_INFO, "Trying to enable keyboard backlight\n"));
  GpioPinWrite (3, GPIO_PIN_PB1, TRUE);
  GpioPinSetDirection (3, GPIO_PIN_PB1, GPIO_PIN_OUTPUT);
}

VOID
EFIAPI
Usb2PhyResume (
  VOID
  )
{
  MmioWrite32(0xfd5d0008, 0x20000000);
  MmioWrite32(0xfd5d4008, 0x20000000);
  MmioWrite32(0xfd5d8008, 0x20000000);
  MmioWrite32(0xfd5dc008, 0x20000000);
  MmioWrite32(0xfd7f0a10, 0x07000700);
  MmioWrite32(0xfd7f0a10, 0x07000000);
}

VOID
EFIAPI
PcieIoInit (
  UINT32 Segment
  )
{
  /* Set reset and power IO to gpio output mode */
  switch(Segment) {
    case PCIE_SEGMENT_PCIE30X4:
      GpioPinSetDirection (4, GPIO_PIN_PD6, GPIO_PIN_OUTPUT);
      GpioPinSetDirection (3, GPIO_PIN_PC6, GPIO_PIN_OUTPUT);
      // PciePinmuxInit(Segment, 1);
      break;
    case PCIE_SEGMENT_PCIE20L0:
      GpioPinSetDirection (4, GPIO_PIN_PA5, GPIO_PIN_OUTPUT);
      // PciePinmuxInit(Segment, 1); // PCIE30x1_0_{CLKREQN,WAKEN,PERSTN}_M1
      break;
    case PCIE_SEGMENT_PCIE20L1:
      break;
    case PCIE_SEGMENT_PCIE20L2:
      break;
    default:
      break;
  }
}

VOID
EFIAPI
PciePowerEn (
  UINT32 Segment,
  BOOLEAN Enable
  )
{
  /* output high to enable power */

  switch(Segment) {
    case PCIE_SEGMENT_PCIE30X4:
      GpioPinWrite (3, GPIO_PIN_PC6, Enable);
      break;
    case PCIE_SEGMENT_PCIE20L0:
      break;
    case PCIE_SEGMENT_PCIE20L1:
      break;
    case PCIE_SEGMENT_PCIE20L2:
      break;
    default:
      break;
  }
}

VOID
EFIAPI
PciePeReset (
  UINT32 Segment,
  BOOLEAN Enable
  )
{
  switch(Segment) {
    case PCIE_SEGMENT_PCIE30X4:
      GpioPinWrite (4, GPIO_PIN_PD6, !Enable);
      break;
    case PCIE_SEGMENT_PCIE20L0: // m.2 a+e key
      GpioPinWrite (4, GPIO_PIN_PA5, !Enable);
      break;
    case PCIE_SEGMENT_PCIE20L1:
      break;
    case PCIE_SEGMENT_PCIE20L2:
      break;
    default:
      break;
  }
}

PWM_DATA pwm_data = {
  .ControllerID = PWM_CONTROLLER3,
  .ChannelID = PWM_CHANNEL2,
  .PeriodNs = 25000,
  .DutyNs = 15000,
  .Polarity = FALSE,
}; // PWM0_CH1

VOID
EFIAPI
PwmFanIoSetup(
  VOID
)
{
  GpioPinSetFunction (1, GPIO_PIN_PD6, 0xB); // PWM14_M2
  RkPwmSetConfig(&pwm_data);
  RkPwmEnable(&pwm_data);
}

VOID
EFIAPI
PwmFanSetSpeed(
  UINT32 Percentage
)
{
  pwm_data.DutyNs = pwm_data.PeriodNs * Percentage / 100;
  RkPwmSetConfig(&pwm_data);
}

VOID
EFIAPI
PlatformInitLeds (
  VOID
  )
{
  /* Status indicator */
}

VOID
EFIAPI
PlatformSetStatusLed (
  IN BOOLEAN Enable
  )
{
  
}

VOID
EFIAPI
PlatformEarlyInit (
  VOID
  )
{
  // Configure various things specific to this platform
  DEBUG((DEBUG_INFO, "PlatformEarlyInit called\n"));
  // GpioPinSetFunction (0, GPIO_PIN_PC0, 0x3); // PWM1_M0
  GpioPinSetFunction (0, GPIO_PIN_PB6, 0x0);
  GpioPinSetFunction (0, GPIO_PIN_PB5, 0x0); // disable UART2_M0

  GpioPinSetFunction (4, GPIO_PIN_PD0, 0xa);
  GpioPinSetFunction (4, GPIO_PIN_PD1, 0xa); // enable UART2_M1

  GpioPinSetDirection(3, GPIO_PIN_PA4, GPIO_PIN_OUTPUT);
  GpioPinWrite(3, GPIO_PIN_PA4, TRUE);

  GpioPinSetDirection(3, GPIO_PIN_PA5, GPIO_PIN_OUTPUT);
  GpioPinWrite(3, GPIO_PIN_PA5, TRUE);

  RkPwmSetConfig(&pwm_data);
  RkPwmEnable(&pwm_data);

  GpioPinSetDirection(1, GPIO_PIN_PB2, GPIO_PIN_INPUT);
  GpioPinSetPull(1, GPIO_PIN_PB2, GPIO_PIN_PULL_UP);
}
