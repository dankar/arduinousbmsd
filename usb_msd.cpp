#include <Spi.h>
#include <Max3421e.h>
#include <Usb.h>

#define DISK_ADDR      1
#define DISK_VID_LO    0x08
#define DISK_VID_HI    0x20
#define DISK_PID_LO    0x18
#define DISK_PID_HI    0x20
#define DISK_CONFIGURATION 1
#define DISK_IF        0
#define DISK_NUM_EP    3
#define EP_MAX_PKTSIZE   0x40
#define EP_INTERRUPT     0x03
#define EP_POLL          0x01

#define CONTROL_EP        0
#define INPUT_EP          1
#define OUTPUT_EP         2

#define USB_MSD_DCBWSIGNATURE               0x43425355ul    // Signature value for the CBW.
#define USB_MSD_DCSWSIGNATURE               0x53425355ul    // Signature value for the CSW.

//#define DEBUG

char buf[ 0x40 ] = { 0 };

#define USB_REQUEST_MASS_STORAGE_RESET    0xff
#define USB_REQUEST_MAX_LUN               0xfe

struct CBW
{
  unsigned long dCBWSignature; // 0x43425355
  unsigned long dCBWTag;
  unsigned long dCBWDataTransferLength;
  byte          dCBWFlags;
  byte          bCBWLUN; // ONLY USE LOW 4 BITS
  byte          bCBWCBLength; // ONLY USE LOW 5 BITS
  byte          CBWCB[16];
};

struct CSW
{
  unsigned long dCSWSignature; // 0x53425355
  unsigned long dCSWTag;
  unsigned long dCSWDataResidue;
  byte          bCSWStatus; //0x00 Success, 0x01 Failed, 0x02 Phase error, 0x03-0xff reserved
};

struct SCSICDB
{
  byte Command[10];
};


#define IN 1
#define OUT 0

class MassStorage
{
private:
  EP_RECORD ep_record[ DISK_NUM_EP ]; 
  MAX3421E Max;
  USB Usb;
  bool m_IsAttached;
  byte m_MaxLUN;
  unsigned long m_CBWTag;
public:

  void PowerOn()
  {
    Max.powerOn();
    delay(200);
    m_IsAttached = false;
    m_MaxLUN = 0;
    m_CBWTag = 1; // For Tags we just keep a number and increment it for each command to the MSD.
  }
  
  bool IsStalled(int endpoint)
  {
    byte rcode;
    
    byte ep_status[2];
    
    rcode = Usb.ctrlReq(DISK_ADDR, ep_record[CONTROL_EP].epAddr, 0x82, 0x00, 0x00, 0x00, endpoint, 2, (char*)ep_status); // Check if the current endpoint is stalled.
    
    if(rcode)
    {
#ifdef DEBUG
      Serial.print("Could not get status on endpoint ");
      Serial.print(endpoint, HEX);
      Serial.print(" Error code: ");
#endif
      Serial.println(rcode, HEX);
      while(1);

    }  
    
    return ep_status[1];
  }
  
  bool CheckStall()
  {
    if(IsStalled(0x81))
    {
#ifdef DEBUG
      Serial.println("INPUT_EP is stalled.");
#endif
      return false; 
    }
    if(IsStalled(0x01))
    {
#ifdef DEBUG
      Serial.println("OUTPUT_EP is stalled.");
#endif
      return false;
    }
    return true;
  }
  
  void PrintResponse(CSW response) // Prints the data in the CSW response.
  {
    Serial.print("Response from mass storage. Signature: ");
    Serial.print(response.dCSWSignature, HEX);
    Serial.print(" Tag: ");
    Serial.print(response.dCSWTag, HEX);
    Serial.print(" Status: ");
    Serial.println(response.bCSWStatus, HEX);
  }
  
  byte Inquiry()
  {
    byte rcode;
    CBW command = {0};
    CSW response = {0};
    SCSICDB scsi_command = {0};
    
    command.dCBWSignature = USB_MSD_DCBWSIGNATURE;
    command.dCBWTag = m_CBWTag++;
    command.dCBWDataTransferLength = 0x24;
    command.dCBWFlags = 0x80;
    command.bCBWLUN = 0;
    command.bCBWCBLength = 0x6;

    scsi_command.Command[0] = 0x12; //INQUIRY
    scsi_command.Command[4] = 0x24;
    
    memcpy(command.CBWCB, &scsi_command, 0x6);
    
    rcode = Usb.outTransfer(DISK_ADDR, ep_record[OUTPUT_EP].epAddr, sizeof(command), (char*)&command);
    
    if(rcode)
      return rcode;
    
    rcode = Usb.inTransfer(DISK_ADDR, ep_record[INPUT_EP].epAddr, 0x24, (char*)buf);

    if(rcode)
      return rcode;
    
    rcode = Usb.inTransfer(DISK_ADDR, ep_record[INPUT_EP].epAddr, sizeof(response), (char*)&response);
    
    if(rcode)
      return rcode;
      
#ifdef DEBUG
    Serial.print("INQUIRY: ");
    for(int i = 0; i < 0x24; i++)
    {
      if(buf[i] >= 32 && buf[i] <= 126)
      {
        Serial.print(buf[i]);  
      }
    }
    Serial.println("\n");
    
    PrintResponse(response);
#endif
    return 0x0;
  }  

  bool Read(unsigned long sector, byte* data)
  {
    byte rcode;
    CBW command = {0};
    CSW response = {0};
    SCSICDB scsi_command = {0};
    
    command.dCBWSignature = USB_MSD_DCBWSIGNATURE;
    command.dCBWTag = m_CBWTag++;
    command.dCBWDataTransferLength = 0x200;
    command.dCBWFlags = 0x80; // Should be direction at bit 7, 0 = host to device, 0x80 = device to host.
    command.bCBWLUN = 0; // We only support logical unit 0
    command.bCBWCBLength = 10; //sizeof(scsi_command);

    scsi_command.Command[0] = 0x28;                  // READ (10) command
    scsi_command.Command[5] = sector & 0xFF;         // 4 byte of LBA to read
    scsi_command.Command[4] = (sector >>= 8) & 0xFF;
    scsi_command.Command[3] = (sector >>= 8) & 0xFF;
    scsi_command.Command[2] = (sector >>= 8) & 0xFF;
    scsi_command.Command[8] = 0x1;                   // Number of LBAs to read.
    
    memcpy(command.CBWCB, &scsi_command, 0xA);

#ifdef DEBUG
    Serial.print("SCSI command: ");

    for(int i = 0; i < 0xA; i++)
    {
      Serial.print(scsi_command.Command[i], HEX);
      Serial.print(" ");
    }
    Serial.println("");
    
    CheckStall();
#endif
    
    rcode = Usb.outTransfer(DISK_ADDR, ep_record[OUTPUT_EP].epAddr, sizeof(command), (char*)&command);
    
    if(rcode)
    {
#ifdef DEBUG
      Serial.print("outTransfer failed: ");
      Serial.println(rcode, HEX);
#endif
      return false;
    }

    rcode = Usb.inTransfer(DISK_ADDR, ep_record[INPUT_EP].epAddr, 0x200, (char*)data);
   
    if(rcode)
    {
#ifdef DEBUG
      Serial.print("Failed getting data, error: ");
      Serial.println(rcode, HEX);
#endif
      return false;
    }
    
    rcode = Usb.inTransfer(DISK_ADDR, ep_record[INPUT_EP].epAddr, sizeof(response), (char*)&response);
    
    if(rcode)
    {
#ifdef DEBUG
      Serial.print("inTransfer failed: ");
      Serial.println(rcode, HEX);
#endif
      
      return false;
    }
    
#ifdef DEBUG
    CheckStall();
    
    PrintResponse(response);
#endif
    
    return response.bCSWStatus ? false : true;
  }
  
  bool Write(unsigned long sector, byte* data)
  {
    byte rcode;
    CBW command = {0};
    CSW response = {0};
    SCSICDB scsi_command = {0};
    
    command.dCBWSignature = USB_MSD_DCBWSIGNATURE;
    command.dCBWTag = m_CBWTag++;
    command.dCBWDataTransferLength = 0x200;
    command.dCBWFlags = 0x00; // Should be direction at bit 7, 0 = host to device, 0x80 = device to host.
    command.bCBWLUN = 0; // We only support logical unit 0
    command.bCBWCBLength = 10; //sizeof(scsi_command);

    scsi_command.Command[0] = 0x2A;                  // READ (10) command
    scsi_command.Command[5] = sector & 0xFF;         // 4 byte of LBA to write
    scsi_command.Command[4] = (sector >>= 8) & 0xFF;
    scsi_command.Command[3] = (sector >>= 8) & 0xFF;
    scsi_command.Command[2] = (sector >>= 8) & 0xFF;
    scsi_command.Command[8] = 0x1;                   // Number of LBAs to write.
    
    memcpy(command.CBWCB, &scsi_command, 0xA);

#ifdef DEBUG
    Serial.print("SCSI command: ");

    for(int i = 0; i < 0xA; i++)
    {
      Serial.print(scsi_command.Command[i], HEX);
      Serial.print(" ");
    }
    Serial.println("");
    
    CheckStall();
#endif
    
    rcode = Usb.outTransfer(DISK_ADDR, ep_record[OUTPUT_EP].epAddr, sizeof(command), (char*)&command);
    
    if(rcode)
    {
#ifdef DEBUG
      Serial.print("outTransfer failed: ");
      Serial.println(rcode, HEX);
#endif
      return false;
    }

    rcode = Usb.outTransfer(DISK_ADDR, ep_record[OUTPUT_EP].epAddr, 0x200, (char*)data);
   
    if(rcode)
    {
#ifdef DEBUG
      Serial.print("Failed writing data, error: ");
      Serial.println(rcode, HEX);
#endif
      return false;
    }
    
    rcode = Usb.inTransfer(DISK_ADDR, ep_record[INPUT_EP].epAddr, sizeof(response), (char*)&response);
    
    if(rcode)
    {
#ifdef DEBUG
      Serial.print("inTransfer failed: ");
      Serial.println(rcode, HEX);
#endif
      
      return false;
    }
    
#ifdef DEBUG
    CheckStall();
    
    PrintResponse(response);
#endif
    
    return response.bCSWStatus ? false : true;
  }

  bool ClearStall(int endpoint)
  {
    byte rcode;
    
    rcode = Usb.ctrlReq(DISK_ADDR, ep_record[CONTROL_EP].epAddr, 0x02, USB_REQUEST_CLEAR_FEATURE, 0x00, 0x00, endpoint, 0, NULL);
    
    if(rcode)
    {
#ifdef DEBUG
      Serial.print("Could not clear STALL. Error code: ");
      Serial.println(rcode, HEX);
#endif
      return false;
    }  
    
    return true;
  }
  
  byte GetMaxLUN()
  {
    byte rcode;
    byte mLUN;
    
    rcode = Usb.ctrlReq(DISK_ADDR, ep_record[CONTROL_EP].epAddr, 0xa1, USB_REQUEST_MAX_LUN, 0x00, 0x00, DISK_IF, 1, (char*)&mLUN);
    
    // There should be error handling here. Devices that do not support more than one logical unit may STALL.
    if(rcode)
    {
#ifdef DEBUG
      Serial.print("Could not get max logical units from device. Return code: ");
      Serial.println(rcode, HEX);
#endif
      return 1;
    }
    
    Serial.print("Max LUN: ");
    Serial.println(mLUN, HEX);
    
    if(mLUN > 0)
    {
#ifdef DEBUG
      Serial.println("We do not support MaxLUN > 0. Stopping.");
#endif
      
      return 1;
    }

    return 0;
  }
  
    
  bool Reset() // Resets Bulk mass transfer and clears STALL flag on data endpoints
  {
    byte rcode;

    rcode = Usb.ctrlReq(DISK_ADDR, ep_record[CONTROL_EP].epAddr, 0x21, USB_REQUEST_MASS_STORAGE_RESET, 0x00, 0x00, DISK_IF, 0, NULL);
    
    if(rcode)
    {
#ifdef DEBUG
      Serial.print("Error resetting mass storage. Return code: ");
      Serial.println(rcode, HEX);
#endif
      return false;
    }

    
    if(!ClearStall(0x81))
      return false;
    
    if(!ClearStall(0x01))
      return false;
  
    
    if(Inquiry())
      return false;
      
    return true;
  }
  
  bool Init()
  {
    byte rcode = 0;
    byte i;
    
    ep_record[CONTROL_EP] = *(Usb.getDevTableEntry(0,0));
    
    ep_record[INPUT_EP].epAddr = 0x01;
    ep_record[INPUT_EP].Attr = 0x02; // Bulk
    ep_record[INPUT_EP].MaxPktSize = 0x40;
    ep_record[INPUT_EP].Interval = 0;
    ep_record[INPUT_EP].sndToggle = bmSNDTOG0;
    ep_record[INPUT_EP].rcvToggle = bmRCVTOG0;
    
    ep_record[OUTPUT_EP].epAddr = 0x01;
    ep_record[OUTPUT_EP].Attr = 0x02; // Bulk
    ep_record[OUTPUT_EP].MaxPktSize = 0x40;
    ep_record[OUTPUT_EP].Interval = 0;
    ep_record[OUTPUT_EP].sndToggle = bmSNDTOG0;
    ep_record[OUTPUT_EP].rcvToggle = bmRCVTOG0;
    
    Usb.setDevTableEntry(DISK_ADDR, ep_record);
    
    rcode = Usb.getDevDescr(DISK_ADDR, ep_record[CONTROL_EP].epAddr, DEV_DESCR_LEN, buf);
    
    if(rcode)
    {
#ifdef DEBUG
      Serial.print("Error reading device descriptor. Return code: ");
      Serial.println(rcode);
#endif

      return false;
    }
    if((buf[ 8 ] != DISK_VID_LO) || (buf[ 9 ] != DISK_VID_HI) || (buf[ 10 ] != DISK_PID_LO) || (buf[ 11 ] != DISK_PID_HI) ) 
    {
#ifdef DEBUG
      Serial.println("Unsupported USB Device");
#endif
      return false;
    }
    
    rcode = Usb.setConf(DISK_ADDR, ep_record[CONTROL_EP].epAddr, DISK_CONFIGURATION);
    
    if(rcode)
    {
 #ifdef DEBUG
      Serial.print("Error configuring USB Mass Storage. Return code: ");
      Serial.println(rcode, HEX);  
 #endif
      return false;
    }
    
    if(Inquiry())
      return false;
    
#ifdef DEBUG
    Serial.println("Device configured.");
#endif
    
    m_MaxLUN = GetMaxLUN();
    
    if(m_MaxLUN)
      return false;
    
    return true;
  }
  
  bool Setup()
  {
    if(Usb.getUsbTaskState() == USB_STATE_CONFIGURING)
    {
      if(Init())
      {
        Usb.setUsbTaskState(USB_STATE_RUNNING);
        m_IsAttached = true;
        return true;
      }
    }
    if(Usb.getUsbTaskState() == USB_STATE_RUNNING)
    {
      return true;
    }
    
    return false;
  }
  
  bool Task()
  {
    Max.Task();
    Usb.Task();
    return true;
  }
  
  bool IsAttached()
  {
    return m_IsAttached;
  }
}Disk;  
    

void setup()
{
  Serial.begin(115200);
  Disk.PowerOn();
}

void loop()
{ 
  byte data[0x200] = {0};

  Disk.Task();
  
  if(!Disk.IsAttached())
  {
    Disk.Setup();
  }
  else
  {
    Disk.Reset();
    
    if(!Disk.CheckStall())
    {
      Serial.println("Some EP is still stalled...");
      while(1);
    }
    Serial.println("Calling write...");
    
    for(int i = 0; i < 500; i++)
    {
      if(!Disk.Write(i, data))
      {
        Serial.println("Write failed...");
      }
    }
    
    memcpy(data, "Hello, world!", 13);
    
    Disk.Write(0, data);
    
    memset(data, 0x0, 13);
    
    Disk.Read(0, data);
    
    Serial.println((char*)data);

    while(1);
  }
}
