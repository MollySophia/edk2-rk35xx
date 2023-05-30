/** @file
*
*  Copyright (c) 2021, Rockchip Limited. All rights reserved.
*
*  SPDX-License-Identifier: BSD-2-Clause-Patent
*
**/
#include <Uefi.h>
#include <Library/ArmLib.h>
#include <Protocol/Cpu.h>
#include <Library/CacheMaintenanceLib.h>
#include <Library/DebugLib.h>
#include <Library/DevicePathLib.h>
#include <Library/IoLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PcdLib.h>
#include <Library/UefiBootManagerLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiRuntimeServicesTableLib.h>
#include <Library/HiiLib.h>
#include <Library/UefiLib.h>
#include <Library/RK806.h>
#include <Library/AcpiLib.h>
#include <Protocol/DevicePath.h>
#include <Protocol/DevicePathFromText.h>
#include <Protocol/EmbeddedGpio.h>
#include <Protocol/LoadedImage.h>
#include <Protocol/PlatformBootManager.h>
#include <Protocol/PlatformVirtualKeyboard.h>
#include <Protocol/AndroidBootImg.h>
#include <Library/DxeServicesTableLib.h>
#include <Library/NonDiscoverableDeviceRegistrationLib.h>
#include <Library/CruLib.h>
#include <Protocol/NonDiscoverableDevice.h>
#include <VarStoreData.h>
#include <Soc.h>
#include <RK3588RegsPeri.h>

#include "RK3588Dxe.h"
#include "RK3588DxeFormSetGuid.h"
#include "CpuPerformance.h"

#define CP_UNCONNECTED    0x0
#define CP_PCIE           0x01
#define CP_SATA           0x10
#define CP_USB3           0x20

#define SATA_CAP            0x0000
#define  SATA_CAP_SSS       BIT27
#define SATA_PI             0x000C
#define SATA_CMD            0x0118
#define  SATA_CMD_FBSCP     BIT22

extern UINT8 RK3588DxeHiiBin[];
extern UINT8 RK3588DxeStrings[];

typedef struct {
  VENDOR_DEVICE_PATH VendorDevicePath;
  EFI_DEVICE_PATH_PROTOCOL End;
} HII_VENDOR_DEVICE_PATH;

STATIC HII_VENDOR_DEVICE_PATH mVendorDevicePath = {
  {
    {
      HARDWARE_DEVICE_PATH,
      HW_VENDOR_DP,
      {
        (UINT8)(sizeof (VENDOR_DEVICE_PATH)),
        (UINT8)((sizeof (VENDOR_DEVICE_PATH)) >> 8)
      }
    },
    RK3588DXE_FORMSET_GUID
  },
  {
    END_DEVICE_PATH_TYPE,
    END_ENTIRE_DEVICE_PATH_SUBTYPE,
    {
      (UINT8)(END_DEVICE_PATH_LENGTH),
      (UINT8)((END_DEVICE_PATH_LENGTH) >> 8)
    }
  }
};

STATIC
EFI_STATUS
EFIAPI
InstallHiiPages (
  VOID
  )
{
  EFI_STATUS     Status;
  EFI_HII_HANDLE HiiHandle;
  EFI_HANDLE     DriverHandle;

  DriverHandle = NULL;
  Status = gBS->InstallMultipleProtocolInterfaces (&DriverHandle,
                  &gEfiDevicePathProtocolGuid,
                  &mVendorDevicePath,
                  NULL);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  HiiHandle = HiiAddPackages (&gRK3588DxeFormSetGuid,
                DriverHandle,
                RK3588DxeStrings,
                RK3588DxeHiiBin,
                NULL);

  if (HiiHandle == NULL) {
    gBS->UninstallMultipleProtocolInterfaces (DriverHandle,
           &gEfiDevicePathProtocolGuid,
           &mVendorDevicePath,
           NULL);
    return EFI_OUT_OF_RESOURCES;
  }
  return EFI_SUCCESS;
}

STATIC 
EFI_STATUS
EFIAPI
SetupVariables (
  VOID
  )
{
  SetupCpuPerfVariables();

  return EFI_SUCCESS; 
}

STATIC
VOID
EFIAPI
ApplyVariables (
  VOID
  )
{
  ApplyCpuClockVariables();
}

STATIC
VOID
UartInit (
  IN VOID
  )
{
  //UINT32     Val;

  DEBUG((EFI_D_WARN, "RK3588InitPeripherals: UartInit()\n"));
  /* make UART1 out of reset */
  //MmioWrite32 (CRU_BASE + SC_PERIPH_RSTDIS3, PERIPH_RST3_UART1);
  //MmioWrite32 (CRU_BASE + SC_PERIPH_CLKEN3, PERIPH_RST3_UART1);
  /* make UART2 out of reset */
  //MmioWrite32 (CRU_BASE + SC_PERIPH_RSTDIS3, PERIPH_RST3_UART2);
  //MmioWrite32 (CRU_BASE + SC_PERIPH_CLKEN3, PERIPH_RST3_UART2);
  /* make UART3 out of reset */
  //MmioWrite32 (CRU_BASE + SC_PERIPH_RSTDIS3, PERIPH_RST3_UART3);
  //MmioWrite32 (CRU_BASE + SC_PERIPH_CLKEN3, PERIPH_RST3_UART3);
  /* make UART4 out of reset */
  //MmioWrite32 (CRU_BASE + SC_PERIPH_RSTDIS3, PERIPH_RST3_UART4);
  //MmioWrite32 (CRU_BASE + SC_PERIPH_CLKEN3, PERIPH_RST3_UART4);

  /* make DW_MMC2 out of reset */
  //MmioWrite32 (CRU_BASE + SC_PERIPH_RSTDIS0, PERIPH_RST0_MMC2);

  /* enable clock for BT/WIFI */
  //Val = MmioRead32 (PMUSSI_ONOFF8_REG) | PMUSSI_ONOFF8_EN_32KB;
  //MmioWrite32 (PMUSSI_ONOFF8_REG, Val);
}

STATIC EMBEDDED_GPIO        *mGpio;

STATIC
VOID
MtcmosInit (
  IN VOID
  )
{
  //UINT32     Data;

  DEBUG((EFI_D_WARN, "RK3588InitPeripherals: MtcmosInit()\n"));
  /* enable MTCMOS for GPU */
  //MmioWrite32 (AO_CTRL_BASE + SC_PW_MTCMOS_EN0, PW_EN0_G3D);
  //do {
  //  Data = MmioRead32 (AO_CTRL_BASE + SC_PW_MTCMOS_ACK_STAT0);
  //} while ((Data & PW_EN0_G3D) == 0);
}

STATIC
VOID
GmacIomuxInit (
  IN VOID
  )
{
  DEBUG((EFI_D_WARN, "RK3588InitPeripherals: GmacIomuxInit()\n"));
  GmacIomux(0);
}

static UINTN ComPhyReg[3][2] = {
  {0xFEE00000, 0xFD5BC000},
  {0xFEE10000, 0xFD5C0000},
  {0xFEE20000, 0xFD5C4000},
};

static UINTN AhciReg[3] = {
  0xFE210000,
  0xFE220000,
  0xFE230000,
};

UINTN
EFIAPI
InitComPhyConfig(
  UINTN PhyBaseAddr,
  UINTN PhpBaseAddr,
  UINT8 PhyMode
)
{
  UINT32 val;
  DEBUG ((DEBUG_ERROR, "%a reg=%x %x mode = %d\n", __func__, PhyBaseAddr, PhpBaseAddr, PhyMode));

  switch (PhyMode) {
  case CP_PCIE:
    MmioWrite32(PhpBaseAddr + 0x0, 0xFFFF1000);
    MmioWrite32(PhpBaseAddr + 0x4, 0xFFFF4000);
    MmioWrite32(PhpBaseAddr + 0x8, 0xFFFF0101);
    MmioWrite32(PhpBaseAddr + 0xc, 0xFFFF0200);

    /* gate_tx_pck_sel length select work for L1SS */
    MmioWrite32(PhyBaseAddr + 0x74, 0xc0);

    /* PLL KVCO tuning fine */
    val = MmioRead32(PhyBaseAddr + (0x20 << 2));
    val &= ~(0x7 << 2);
    val |= 0x4 << 2;
    MmioWrite32(PhyBaseAddr + (0x20 << 2), val);

    /* Set up rx_trim: PLL LPF C1 85pf R1 1.25kohm */
    MmioWrite32(PhyBaseAddr + (0x1b << 2), 0x4c);

    /* Set up su_trim: T3 */
    MmioWrite32(PhyBaseAddr + (0xa << 2), 0xb0);
    MmioWrite32(PhyBaseAddr + (0xb << 2), 0x47);
    MmioWrite32(PhyBaseAddr + (0xd << 2), 0x57);

    break;

  case CP_SATA:
    MmioWrite32(PhyBaseAddr + 0x38, 0x41);
    MmioWrite32(PhyBaseAddr + 0x18, 0x8F);
    MmioWrite32(PhyBaseAddr + 0x7C, 0x50);
    MmioWrite32(PhyBaseAddr + 0x24, 0x07);

    MmioWrite32(PhpBaseAddr + 0x0, 0xFFFF0129);
    MmioWrite32(PhpBaseAddr + 0x4, 0xFFFF4000);
    MmioWrite32(PhpBaseAddr + 0x8, 0xFFFF80c1);
    MmioWrite32(PhpBaseAddr + 0xc, 0xFFFF0407);

    /* Should we tune the rest of the parameters too? */

    break;

  case CP_USB3:
    /* Set SSC downward spread spectrum */
    val = MmioRead32(PhyBaseAddr + (0x1f << 2));
    val &= ~(0x3 << 4);
    val |= 0x01 << 4;
    MmioWrite32(PhyBaseAddr + 0x7c, val);

    /* Enable adaptive CTLE for USB3.0 Rx */
    val = MmioRead32(PhyBaseAddr + (0x0e << 2));
    val &= ~(0x1 << 0);
    val |= 0x01;
    MmioWrite32(PhyBaseAddr + (0x0e << 2), val);

    /* Set PLL KVCO fine tuning signals */
    val = MmioRead32(PhyBaseAddr + (0x20 << 2));
    val &= ~(0x7 << 2);
    val |= 0x2 << 2;
    MmioWrite32(PhyBaseAddr + (0x20 << 2), val);

    /* Set PLL LPF R1 to su_trim[10:7]=1001 */
    MmioWrite32(PhyBaseAddr + (0xb << 2), 0x4);

    /* Set PLL input clock divider 1/2 */
    val = MmioRead32(PhyBaseAddr + (0x5 << 2));
    val &= ~(0x3 << 6);
    val |= 0x1 << 6;
    MmioWrite32(PhyBaseAddr + (0x5 << 2), val);

    /* Set PLL loop divider */
    MmioWrite32(PhyBaseAddr + (0x11 << 2), 0x32);

    /* Set PLL KVCO to min and set PLL charge pump current to max */
    MmioWrite32(PhyBaseAddr + (0xa << 2), 0xf0);

    /* Set Rx squelch input filler bandwidth */
    MmioWrite32(PhyBaseAddr + (0x14 << 2), 0x0d);

    /* Set txcomp_sel[15] to 1b'0 */
    MmioWrite32(PhpBaseAddr + 0x8, 0x80000000);
    /* Set txelec_sel[12] to 1b'0 */
    MmioWrite32(PhpBaseAddr + 0x8, 0x10000000);
    /* Set usb_mode_set[5:0]=6b'000100 */
    MmioWrite32(PhpBaseAddr + 0x0, 0x003F0004);

    /* phy_clk_sel to 100MHz */
    MmioWrite32(PhpBaseAddr + 0x4, 0x60004000);
    break;

  default:
    break;
  }

  return RETURN_SUCCESS;
}

UINTN
EFIAPI
ComboPhyInit(void)
{
  UINT32 data;
  UINT8 *ComPhyMode;
  UINTN ComPhyDeviceTableSize, Index;

  ComPhyMode = PcdGetPtr (PcdComboPhyMode);
  if (ComPhyMode == NULL)
    return RETURN_NOT_FOUND;

  ComPhyDeviceTableSize = PcdGetSize (PcdComboPhyMode);

  /* config phy clock to 100Mhz */
  HAL_CRU_ClkSetFreq(PLL_PPLL, 100 * 1000000);
  HAL_CRU_ClkSetFreq(CLK_REF_PIPE_PHY0, 100 * 1000000);
  HAL_CRU_ClkSetFreq(CLK_REF_PIPE_PHY1, 100 * 1000000);
  HAL_CRU_ClkSetFreq(CLK_REF_PIPE_PHY2, 100 * 1000000);

  /* Initialize echo combo phy */
  for (Index = 0; Index < ComPhyDeviceTableSize; Index++) {
    InitComPhyConfig (ComPhyReg[Index][0], ComPhyReg[Index][1], ComPhyMode[Index]);
    if (ComPhyMode[Index] == CP_SATA) {

    /* Set port implemented flag */
    MmioWrite32 (AhciReg[Index] + SATA_PI, 0x1);

    /* Supports staggered spin-up */
    MmioOr32 (AhciReg[Index] + SATA_CAP, SATA_CAP_SSS);

    /* Supports FIS-based switching */
    MmioOr32 (AhciReg[Index] + SATA_CMD, SATA_CMD_FBSCP);

		RegisterNonDiscoverableMmioDevice (
           NonDiscoverableDeviceTypeAhci,
           NonDiscoverableDeviceDmaTypeNonCoherent,
           NULL,
           NULL,
           1,
           AhciReg[Index], SIZE_4KB);
    }
  }
  /* Initialize SATA */
  data = MmioRead32(0xfe210000 + 0x118);
  MmioWrite32(0xfe210000 + 0x118, data | 1 << 22); /* FBSCP */
  MmioWrite32(0xfe210000 + 0xc, 1); /* HOST_PORTS_IMPL */

  data = MmioRead32(0xfe220000 + 0x118);
  MmioWrite32(0xfe220000 + 0x118, data | 1 << 22); /* FBSCP */
  MmioWrite32(0xfe220000 + 0xc, 1); /* HOST_PORTS_IMPL */

  data = MmioRead32(0xfe230000 + 0x118);
  MmioWrite32(0xfe230000 + 0x118, data | 1 << 22); /* FBSCP */
  MmioWrite32(0xfe230000 + 0xc, 1); /* HOST_PORTS_IMPL */

  MmioWrite32(0xfd5b0000 + 0x0, 0x07E00440);
  MmioWrite32(0xfd5b0000 + 0x4, 0x00070002);

  /* reset deassert */
  MmioWrite32(0xfd7c0000 + 0x0b34, 0x01c00000);

  for (Index = 0; Index < ComPhyDeviceTableSize; Index++)
    if (ComPhyMode[Index] == CP_USB3)
      gBS->Stall(2000);

  return RETURN_SUCCESS;
}

EFI_STATUS
RK3588InitPeripherals (
  IN VOID
  )
{
  INT32 i;
  //UINT32     Data, Bits;

  DEBUG((EFI_D_WARN, "RK3588InitPeripherals: Entry\n"));

  /* make I2C0/I2C1/I2C2/SPI0 out of reset */
  //Bits = PERIPH_RST3_I2C0 | PERIPH_RST3_I2C1 | PERIPH_RST3_I2C2 | PERIPH_RST3_SSP;
  //MmioWrite32 (CRU_BASE + SC_PERIPH_RSTDIS3, Bits);

  //do {
  //  Data = MmioRead32 (CRU_BASE + SC_PERIPH_RSTSTAT3);
  //} while (Data & Bits);

  // UartInit ();

  /* MTCMOS -- Multi-threshold CMOS */
  // MtcmosInit ();

  /* Set DETECT_J15_FASTBOOT (GPIO24) pin as GPIO function */
  //MmioWrite32 (IOCG_084_REG, 0);        /* configure GPIO24 as nopull */
  //MmioWrite32 (IOMG_080_REG, 0);        /* configure GPIO24 as GPIO */

  Rk806Configure();

  ComboPhyInit();

  GmacIomuxInit();
  
  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
CreatePlatformBootOptionFromPath (
  IN     CHAR16                          *PathStr,
  IN     CHAR16                          *Description,
  IN OUT EFI_BOOT_MANAGER_LOAD_OPTION    *BootOption
  )
{
  EFI_STATUS                   Status;
  EFI_DEVICE_PATH              *DevicePath;

  DevicePath = (EFI_DEVICE_PATH *)ConvertTextToDevicePath (PathStr);
  ASSERT (DevicePath != NULL);
  Status = EfiBootManagerInitializeLoadOption (
             BootOption,
             LoadOptionNumberUnassigned,
             LoadOptionTypeBoot,
             LOAD_OPTION_ACTIVE,
             Description,
             DevicePath,
             NULL,
             0
             );
  FreePool (DevicePath);
  return Status;
}

STATIC
EFI_STATUS
CreatePlatformBootOptionFromGuid (
  IN     EFI_GUID                        *FileGuid,
  IN     CHAR16                          *Description,
  IN OUT EFI_BOOT_MANAGER_LOAD_OPTION    *BootOption
  )
{
  EFI_STATUS                             Status;
  EFI_DEVICE_PATH                        *DevicePath;
  EFI_DEVICE_PATH                        *TempDevicePath;
  EFI_LOADED_IMAGE_PROTOCOL              *LoadedImage;
  MEDIA_FW_VOL_FILEPATH_DEVICE_PATH      FileNode;

  Status = gBS->HandleProtocol (
                  gImageHandle,
                  &gEfiLoadedImageProtocolGuid,
                  (VOID **) &LoadedImage
                  );
  ASSERT_EFI_ERROR (Status);
  EfiInitializeFwVolDevicepathNode (&FileNode, FileGuid);
  TempDevicePath = DevicePathFromHandle (LoadedImage->DeviceHandle);
  ASSERT (TempDevicePath != NULL);
  DevicePath = AppendDevicePathNode (
                 TempDevicePath,
                 (EFI_DEVICE_PATH_PROTOCOL *) &FileNode
                 );
  ASSERT (DevicePath != NULL);
  Status = EfiBootManagerInitializeLoadOption (
             BootOption,
             LoadOptionNumberUnassigned,
             LoadOptionTypeBoot,
             LOAD_OPTION_ACTIVE,
             Description,
             DevicePath,
             NULL,
             0
             );
  FreePool (DevicePath);
  return Status;
}

STATIC
EFI_STATUS
GetPlatformBootOptionsAndKeys (
  OUT UINTN                              *BootCount,
  OUT EFI_BOOT_MANAGER_LOAD_OPTION       **BootOptions,
  OUT EFI_INPUT_KEY                      **BootKeys
  )
{
  EFI_GUID                               *FileGuid;
  CHAR16                                 *PathStr;
  EFI_STATUS                             Status;
  UINTN                                  Size;

  *BootCount = 4;

  Size = sizeof (EFI_BOOT_MANAGER_LOAD_OPTION) * *BootCount;
  *BootOptions = (EFI_BOOT_MANAGER_LOAD_OPTION *)AllocateZeroPool (Size);
  if (*BootOptions == NULL) {
    DEBUG ((DEBUG_ERROR, "Failed to allocate memory for BootOptions\n"));
    return EFI_OUT_OF_RESOURCES;
  }
  Size = sizeof (EFI_INPUT_KEY) * *BootCount;
  *BootKeys = (EFI_INPUT_KEY *)AllocateZeroPool (Size);
  if (*BootKeys == NULL) {
    DEBUG ((DEBUG_ERROR, "Failed to allocate memory for BootKeys\n"));
    Status = EFI_OUT_OF_RESOURCES;
    goto Error;
  }

  PathStr = (CHAR16 *)PcdGetPtr (PcdSdBootDevicePath);
  ASSERT (PathStr != NULL);
  Status = CreatePlatformBootOptionFromPath (
             PathStr,
             L"Boot from SD",
             &(*BootOptions)[0]
             );
  ASSERT_EFI_ERROR (Status);

  PathStr = (CHAR16 *)PcdGetPtr (PcdAndroidBootDevicePath);
  ASSERT (PathStr != NULL);
  Status = CreatePlatformBootOptionFromPath (
             PathStr,
             L"Grub",
             &(*BootOptions)[1]
             );
  ASSERT_EFI_ERROR (Status);

  FileGuid = PcdGetPtr (PcdAndroidBootFile);
  ASSERT (FileGuid != NULL);
  Status = CreatePlatformBootOptionFromGuid (
             FileGuid,
             L"Android Boot",
             &(*BootOptions)[2]
             );
  ASSERT_EFI_ERROR (Status);

  FileGuid = PcdGetPtr (PcdAndroidFastbootFile);
  ASSERT (FileGuid != NULL);
  Status = CreatePlatformBootOptionFromGuid (
             FileGuid,
             L"Android Fastboot",
             &(*BootOptions)[3]
             );
  ASSERT_EFI_ERROR (Status);
  (*BootKeys)[3].ScanCode = SCAN_NULL;
  (*BootKeys)[3].UnicodeChar = 'f';

  return EFI_SUCCESS;
Error:
  FreePool (*BootOptions);
  return Status;
}

PLATFORM_BOOT_MANAGER_PROTOCOL mPlatformBootManager = {
  GetPlatformBootOptionsAndKeys
};

EFI_STATUS
EFIAPI
VirtualKeyboardRegister (
  IN VOID
  )
{
  EFI_STATUS           Status;

  Status = gBS->LocateProtocol (
                  &gEmbeddedGpioProtocolGuid,
                  NULL,
                  (VOID **) &mGpio
                  );
  if (EFI_ERROR (Status)) {
    return Status;
  }
  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
VirtualKeyboardReset (
  IN VOID
  )
{
  EFI_STATUS           Status;

  if (mGpio == NULL) {
    return EFI_INVALID_PARAMETER;
  }
  Status = mGpio->Set (mGpio, DETECT_J15_FASTBOOT, GPIO_MODE_INPUT);
  return Status;
}

BOOLEAN
EFIAPI
VirtualKeyboardQuery (
  IN VIRTUAL_KBD_KEY             *VirtualKey
  )
{
  EFI_STATUS           Status;
  UINTN                Value = 0;

  if ((VirtualKey == NULL) || (mGpio == NULL)) {
    return FALSE;
  }
  if (MmioRead32 (ADB_REBOOT_ADDRESS) == ADB_REBOOT_BOOTLOADER) {
    goto Done;
  } else {
    Status = mGpio->Get (mGpio, DETECT_J15_FASTBOOT, &Value);
    if (EFI_ERROR (Status) || (Value != 0)) {
      return FALSE;
    }
  }
Done:
  VirtualKey->Signature = VIRTUAL_KEYBOARD_KEY_SIGNATURE;
  VirtualKey->Key.ScanCode = SCAN_NULL;
  VirtualKey->Key.UnicodeChar = L'f';
  return TRUE;
}

EFI_STATUS
EFIAPI
VirtualKeyboardClear (
  IN VIRTUAL_KBD_KEY            *VirtualKey
  )
{
  if (VirtualKey == NULL) {
    return EFI_INVALID_PARAMETER;
  }
  if (MmioRead32 (ADB_REBOOT_ADDRESS) == ADB_REBOOT_BOOTLOADER) {
    MmioWrite32 (ADB_REBOOT_ADDRESS, ADB_REBOOT_NONE);
    WriteBackInvalidateDataCacheRange ((VOID *)ADB_REBOOT_ADDRESS, 4);
  }
  return EFI_SUCCESS;
}

PLATFORM_VIRTUAL_KBD_PROTOCOL mVirtualKeyboard = {
  VirtualKeyboardRegister,
  VirtualKeyboardReset,
  VirtualKeyboardQuery,
  VirtualKeyboardClear
};

STATIC  EFI_STATUS EFIAPI AppendArgs (
  IN CHAR16            *Args,
  IN UINTN              Size
  )
{
	CHAR16 *newArgs =   (CHAR16 *)PcdGetPtr (PcdKernelBootArg);
    UINTN srcSize, i, bootArgSize;

    for (srcSize = 0; srcSize < Size / 2; srcSize++) {
        if (!Args[srcSize])
			break;
    }

    for (bootArgSize = 0; bootArgSize < Size / 2; bootArgSize++) {
        if (!newArgs[bootArgSize])
			break;
    }

	if (bootArgSize * 2 + srcSize * 2 < Size)
	    for (i = 0; i < bootArgSize; i++)
            Args[i] = newArgs[i];

    return 0;
}

ANDROID_BOOTIMG_PROTOCOL mAndroidBootImageManager = {
	  AppendArgs,
	  NULL
};

STATIC CONST EFI_GUID mAcpiTableFile = {
  0x7E374E25, 0x8E01, 0x4FEE, { 0x87, 0xf2, 0x39, 0x0C, 0x23, 0xC6, 0x06, 0xCD }
};

STATIC VOID SetFlashAttributeToUncache(VOID)
{
  EFI_STATUS Status;
  EFI_GCD_MEMORY_SPACE_DESCRIPTOR desp = {0};

  Status = gDS->AddMemorySpace (
                     EfiGcdMemoryTypeMemoryMappedIo,
                     PcdGet64(FspiBaseAddr),
                     SIZE_64KB,
                     EFI_MEMORY_UC | EFI_MEMORY_RUNTIME
                     );
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "RTC: Failed to add memory space Status = %r\n", Status));
    return;
  }

  Status = gDS->GetMemorySpaceDescriptor(PcdGet64(FspiBaseAddr),&desp);
  DEBUG ((DEBUG_ERROR, "%a: GetMemorySpaceDescriptor status = %x\n", __FUNCTION__, Status));
  if(EFI_ERROR(Status)){
    return;
  }

  Status = gDS->SetMemorySpaceAttributes (
                     PcdGet64(FspiBaseAddr),
                     SIZE_64KB,
                     EFI_MEMORY_UC | EFI_MEMORY_RUNTIME
                     );
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "%a: Failed to set memory attributes Status = %x\n",__FUNCTION__, Status));
  }

  Status = gDS->AddMemorySpace (
                     EfiGcdMemoryTypeMemoryMappedIo,
                     PcdGet64(CruBaseAddr),
                     SIZE_64KB,
                     EFI_MEMORY_UC | EFI_MEMORY_RUNTIME
                     );
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "RTC: Failed to add memory space Status = %r\n", Status));
    return;
  }

  Status = gDS->GetMemorySpaceDescriptor(PcdGet64(CruBaseAddr),&desp);
    DEBUG ((DEBUG_ERROR, "%a: GetMemorySpaceDescriptor status = %x\n", __FUNCTION__, Status));
  if(EFI_ERROR(Status)){
    return;
  }

  Status = gDS->SetMemorySpaceAttributes (
                     PcdGet64(CruBaseAddr),
                     SIZE_64KB,
                     EFI_MEMORY_UC | EFI_MEMORY_RUNTIME
                     );
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "%a: Failed to set memory attributes Status = %x\n",__FUNCTION__, Status));
  }

}

STATIC
VOID
EFIAPI
OnEfiVariableWriteArchRegistrationEvent (
  IN  EFI_EVENT   Event,
  IN  VOID        *Context
  )
{
  EFI_STATUS                  Status;
  EFI_HANDLE                  *HandleBuffer;
  UINTN                       NumProtocols;

  Status = gBS->LocateHandleBuffer (ByProtocol,
                                    &gEfiVariableWriteArchProtocolGuid,
                                    NULL,
                                    &NumProtocols,
                                    &HandleBuffer);
  if (EFI_ERROR(Status)) {
    if (Status != EFI_NOT_FOUND) {
        DEBUG((DEBUG_WARN, "Couldn't locate gEfiVariableWriteArchProtocolGuid. Status=%r\n", Status));
    }
    return; // false alert
  }

  gBS->CloseEvent (Event);

  Status = SetupVariables ();
  if (Status != EFI_SUCCESS) {
    DEBUG ((DEBUG_ERROR, "Couldn't setup NV vars. Status=%r\n", Status));
  }

  ApplyVariables ();

  Status = InstallHiiPages ();
  if (Status != EFI_SUCCESS) {
    DEBUG ((DEBUG_ERROR, "Couldn't install RK3588Dxe HII pages. Status=%r\n", Status));
  }
}

VOID
EFIAPI
RK3588NotifyReadyToBoot (
  IN EFI_EVENT  Event,
  IN VOID       *Context
  )
{
  //
  // We set CPU voltages late on first notification of the ReadyToBoot event because
  // the user should have a chance to dial settings back in case they've set values that are
  // too high and/or the power supply is insufficient. Setting them earlier may prevent this.
  // The default values provide enough performance for the UEFI environment, so there's
  // really no need to set them early.
  //

  DEBUG ((DEBUG_ERROR, "%a: called. Configure CPU voltages once.\n", __FUNCTION__));

  ApplyCpuVoltageVariables ();

  gBS->CloseEvent (Event);
}

EFI_STATUS
EFIAPI
RK3588EntryPoint (
  IN EFI_HANDLE         ImageHandle,
  IN EFI_SYSTEM_TABLE   *SystemTable
  )
{
  EFI_STATUS            Status;
  VOID                  *Rk860xRegulatorRegistration;
  VOID                  *EfiVariableArchRegistrationEvent;
  EFI_EVENT             ReadyToBootEvent;

  PlatformEarlyInit();

  //
  // We actually depend on gEfiVariableWriteArchProtocolGuid but don't want to
  // delay the entire driver, so we create a notify event on protocol arrival instead
  // and set up the variables & HII data in the callback.
  //
  EfiCreateProtocolNotifyEvent (&gEfiVariableWriteArchProtocolGuid,
                                TPL_CALLBACK,
                                OnEfiVariableWriteArchRegistrationEvent,
                                NULL,
                                &EfiVariableArchRegistrationEvent);

  Status = EfiCreateEventReadyToBootEx (TPL_CALLBACK,
                                        RK3588NotifyReadyToBoot,
                                        NULL,
                                        &ReadyToBootEvent);
  ASSERT_EFI_ERROR (Status);

  Status = RK3588InitPeripherals ();
  if (EFI_ERROR (Status)) {
    return Status;
  }

  SetFlashAttributeToUncache();

  if(PcdGetBool (AcpiEnable)) {
    LocateAndInstallAcpiFromFvConditional (&mAcpiTableFile, NULL);
  }

  Status = gBS->InstallProtocolInterface (
                  &ImageHandle,
                  &gPlatformVirtualKeyboardProtocolGuid,
                  EFI_NATIVE_INTERFACE,
                  &mVirtualKeyboard
                  );
  if (EFI_ERROR (Status)) {
    return Status;
  }

  Status = gBS->InstallProtocolInterface (
                  &ImageHandle,
                  &gPlatformBootManagerProtocolGuid,
                  EFI_NATIVE_INTERFACE,
                  &mPlatformBootManager
                  );

  if (EFI_ERROR (Status)) {
    return Status;
  }

  Status = gBS->InstallProtocolInterface (
                  &ImageHandle,
                  &gAndroidBootImgProtocolGuid,
                  EFI_NATIVE_INTERFACE,
                  &mAndroidBootImageManager
                  );

  return Status;
}
