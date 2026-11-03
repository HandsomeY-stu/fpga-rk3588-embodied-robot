/*
 * M24LC04B.v
 * I2C EEPROM仿真模型，用于验证I2C控制器。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
// **     This information is distributed under license from Young Engineering.                         **
// **                              COPYRIGHT (c) 2003 YOUNG ENGINEERING                                 **

`timescale 1ns/10ps

module M24LC04B (A0, A1, A2, WP, SDA, SCL, RESET);

   input  A0;        
   input  A1;        
   input  A2;        

   input  WP;        

   inout  SDA;       
   input  SCL;       

   input  RESET;     

   reg            SDA_DO;                  
   reg            SDA_OE;                  

   wire           SDA_DriveEnable;         
   reg            SDA_DriveEnableDlyd;     

   wire [02:00]   ChipAddress;             

   reg  [03:00]   BitCounter;              

   reg            START_Rcvd;              
   reg            STOP_Rcvd;               
   reg            CTRL_Rcvd;               
   reg            ADDR_Rcvd;               
   reg            MACK_Rcvd;               

   reg            WrCycle;                 
   reg            RdCycle;                 

   reg  [07:00]   ShiftRegister;           

   reg  [07:00]   ControlByte;             
   wire           BlockSelect;             
   wire           RdWrBit;                 

   reg  [08:00]   StartAddress;            
   reg  [03:00]   PageAddress;             

   reg  [07:00]   WrDataByte [0:15];       
   wire [07:00]   RdDataByte;              

   reg  [15:00]   WrCounter;               

   reg  [03:00]   WrPointer;               
   reg  [08:00]   RdPointer;               

   reg            WriteActive;             

   reg  [07:00]   MemoryBlock0 [0:255];    
   reg  [07:00]   MemoryBlock1 [0:255];    

   integer        LoopIndex;               

   integer        tAA;                     
   integer        tWC;                     

   initial tAA = 900;                                    
   initial tWC = 5000000;                                

   initial begin
      SDA_DO = 0;
      SDA_OE = 0;
   end

   initial begin
      START_Rcvd = 0;
      STOP_Rcvd  = 0;
      CTRL_Rcvd  = 0;
      ADDR_Rcvd  = 0;
      MACK_Rcvd  = 0;
   end

   initial begin
      BitCounter  = 0;
      ControlByte = 0;
   end

   initial begin
      WrCycle = 0;
      RdCycle = 0;

      WriteActive = 0;
   end

   assign ChipAddress = {A2,A1,A0};

   always @(negedge SDA) begin
      if (SCL == 1) begin
         START_Rcvd <= 1;
         STOP_Rcvd  <= 0;
         CTRL_Rcvd  <= 0;
         ADDR_Rcvd  <= 0;
         MACK_Rcvd  <= 0;

         WrCycle <= #1 0;
         RdCycle <= #1 0;

         BitCounter <= 0;
      end
   end

   always @(posedge SDA) begin
      if (SCL == 1) begin
         START_Rcvd <= 0;
         STOP_Rcvd  <= 1;
         CTRL_Rcvd  <= 0;
         ADDR_Rcvd  <= 0;
         MACK_Rcvd  <= 0;

         WrCycle <= #1 0;
         RdCycle <= #1 0;

         BitCounter <= 10;
      end
   end

   always @(posedge SCL) begin
      ShiftRegister[00] <= SDA;
      ShiftRegister[01] <= ShiftRegister[00];
      ShiftRegister[02] <= ShiftRegister[01];
      ShiftRegister[03] <= ShiftRegister[02];
      ShiftRegister[04] <= ShiftRegister[03];
      ShiftRegister[05] <= ShiftRegister[04];
      ShiftRegister[06] <= ShiftRegister[05];
      ShiftRegister[07] <= ShiftRegister[06];
   end

   always @(posedge SCL) begin
      if (BitCounter < 10) BitCounter <= BitCounter + 1;
   end

   always @(negedge SCL) begin
      if (START_Rcvd & (BitCounter == 8)) begin
         if (!WriteActive & (ShiftRegister[07:01] == {4'b1010,ChipAddress[02:00]})) begin
            if (ShiftRegister[00] == 0) WrCycle <= 1;
            if (ShiftRegister[00] == 1) RdCycle <= 1;

            ControlByte <= ShiftRegister[07:00];

            CTRL_Rcvd <= 1;
         end

         START_Rcvd <= 0;
      end
   end

   assign BlockSelect = ControlByte[01];
   assign RdWrBit     = ControlByte[00];

   always @(negedge SCL) begin
      if (CTRL_Rcvd & (BitCounter == 8)) begin
         if (RdWrBit == 0) begin
            StartAddress <= {BlockSelect,ShiftRegister[07:00]};
            RdPointer    <= {BlockSelect,ShiftRegister[07:00]};

            ADDR_Rcvd <= 1;
         end

         WrCounter <= 0;
         WrPointer <= 0;

         CTRL_Rcvd <= 0;
      end
   end

   always @(negedge SCL) begin
      if (ADDR_Rcvd & (BitCounter == 8)) begin
         if ((WP == 0) & (RdWrBit == 0)) begin
            WrDataByte[WrPointer] <= ShiftRegister[07:00];

            WrCounter <= WrCounter + 1;
            WrPointer <= WrPointer + 1;
         end
      end
   end

   always @(negedge SCL) begin
      if (!WriteActive) begin
         if (BitCounter == 8) begin
            if (WrCycle | (START_Rcvd & (ShiftRegister[07:01] == {4'b1010,ChipAddress[02:00]}))) begin
               SDA_DO <= 0;
               SDA_OE <= 1;
            end 
         end
         if (BitCounter == 9) begin
            BitCounter <= 0;

            if (!RdCycle) begin
               SDA_DO <= 0;
               SDA_OE <= 0;
            end
         end
      end
   end 

   always @(posedge SCL) begin
      if (RdCycle & (BitCounter == 8)) begin
         if ((SDA == 0) & (SDA_OE == 0)) MACK_Rcvd <= 1;
      end
   end

   always @(negedge SCL) MACK_Rcvd <= 0;

   always @(posedge STOP_Rcvd) begin
      if (WrCycle & (WP == 0) & (WrCounter > 0)) begin
         WriteActive = 1;
         #(tWC);
         WriteActive = 0;
      end
   end

   always @(posedge STOP_Rcvd) begin
      #(1.0);
      STOP_Rcvd = 0;
   end

   always @(negedge WriteActive) begin
      for (LoopIndex = 0; LoopIndex < WrCounter; LoopIndex = LoopIndex + 1) begin
         if (StartAddress[08] == 0) begin
            PageAddress = StartAddress[03:00] + LoopIndex;

            MemoryBlock0[{StartAddress[07:04],PageAddress[03:00]}] = WrDataByte[LoopIndex[03:00]];
         end
         if (StartAddress[08] == 1) begin
            PageAddress = StartAddress[03:00] + LoopIndex;

            MemoryBlock1[{StartAddress[07:04],PageAddress[03:00]}] = WrDataByte[LoopIndex[03:00]];
         end
      end
   end

   always @(negedge SCL) begin
      if (BitCounter == 8) begin
         if (WrCycle & ADDR_Rcvd) begin
            RdPointer <= StartAddress + WrPointer + 1;
         end
         if (RdCycle) begin
            RdPointer <= RdPointer + 1;
         end
      end
   end

   assign RdDataByte = RdPointer[08] ? MemoryBlock1[RdPointer[07:00]] : MemoryBlock0[RdPointer[07:00]];

   always @(negedge SCL) begin
      if (RdCycle) begin
         if (BitCounter == 8) begin
            SDA_DO <= 0;
            SDA_OE <= 0;
         end
         else if (BitCounter == 9) begin
            SDA_DO <= RdDataByte[07];

            if (MACK_Rcvd) SDA_OE <= 1;
         end
         else begin
            SDA_DO <= RdDataByte[7-BitCounter];
         end
      end
   end

   bufif1 (SDA, 1'b0, SDA_DriveEnableDlyd);

   assign SDA_DriveEnable = !SDA_DO & SDA_OE;
   always @(SDA_DriveEnable) SDA_DriveEnableDlyd <= #(tAA) SDA_DriveEnable;

   wire [07:00]  MemoryByte0_00 = MemoryBlock0[00];
   wire [07:00]  MemoryByte0_01 = MemoryBlock0[01];
   wire [07:00]  MemoryByte0_02 = MemoryBlock0[02];
   wire [07:00]  MemoryByte0_03 = MemoryBlock0[03];
   wire [07:00]  MemoryByte0_04 = MemoryBlock0[04];
   wire [07:00]  MemoryByte0_05 = MemoryBlock0[05];
   wire [07:00]  MemoryByte0_06 = MemoryBlock0[06];
   wire [07:00]  MemoryByte0_07 = MemoryBlock0[07];

   wire [07:00]  MemoryByte0_08 = MemoryBlock0[08];
   wire [07:00]  MemoryByte0_09 = MemoryBlock0[09];
   wire [07:00]  MemoryByte0_0A = MemoryBlock0[10];
   wire [07:00]  MemoryByte0_0B = MemoryBlock0[11];
   wire [07:00]  MemoryByte0_0C = MemoryBlock0[12];
   wire [07:00]  MemoryByte0_0D = MemoryBlock0[13];
   wire [07:00]  MemoryByte0_0E = MemoryBlock0[14];
   wire [07:00]  MemoryByte0_0F = MemoryBlock0[15];

   wire [07:00]  MemoryByte1_00 = MemoryBlock1[00];
   wire [07:00]  MemoryByte1_01 = MemoryBlock1[01];
   wire [07:00]  MemoryByte1_02 = MemoryBlock1[02];
   wire [07:00]  MemoryByte1_03 = MemoryBlock1[03];
   wire [07:00]  MemoryByte1_04 = MemoryBlock1[04];
   wire [07:00]  MemoryByte1_05 = MemoryBlock1[05];
   wire [07:00]  MemoryByte1_06 = MemoryBlock1[06];
   wire [07:00]  MemoryByte1_07 = MemoryBlock1[07];

   wire [07:00]  MemoryByte1_08 = MemoryBlock1[08];
   wire [07:00]  MemoryByte1_09 = MemoryBlock1[09];
   wire [07:00]  MemoryByte1_0A = MemoryBlock1[10];
   wire [07:00]  MemoryByte1_0B = MemoryBlock1[11];
   wire [07:00]  MemoryByte1_0C = MemoryBlock1[12];
   wire [07:00]  MemoryByte1_0D = MemoryBlock1[13];
   wire [07:00]  MemoryByte1_0E = MemoryBlock1[14];
   wire [07:00]  MemoryByte1_0F = MemoryBlock1[15];

   wire [07:00]  WriteData_0 = WrDataByte[00];
   wire [07:00]  WriteData_1 = WrDataByte[01];
   wire [07:00]  WriteData_2 = WrDataByte[02];
   wire [07:00]  WriteData_3 = WrDataByte[03];
   wire [07:00]  WriteData_4 = WrDataByte[04];
   wire [07:00]  WriteData_5 = WrDataByte[05];
   wire [07:00]  WriteData_6 = WrDataByte[06];
   wire [07:00]  WriteData_7 = WrDataByte[07];
   wire [07:00]  WriteData_8 = WrDataByte[08];
   wire [07:00]  WriteData_9 = WrDataByte[09];
   wire [07:00]  WriteData_A = WrDataByte[10];
   wire [07:00]  WriteData_B = WrDataByte[11];
   wire [07:00]  WriteData_C = WrDataByte[12];
   wire [07:00]  WriteData_D = WrDataByte[13];
   wire [07:00]  WriteData_E = WrDataByte[14];
   wire [07:00]  WriteData_F = WrDataByte[15];

   wire TimingCheckEnable = (RESET == 0) & (SDA_OE == 0);

   specify
      specparam
         tHI = 600,                                      
         tLO = 1300,                                     
         tSU_STA = 600,                                  
         tHD_STA = 600,                                  
         tSU_DAT = 100,                                  
         tSU_STO = 600,                                  
         tBUF = 1300;                                    

      $width (posedge SCL, tHI);
      $width (negedge SCL, tLO);

      $width (posedge SDA &&& SCL, tBUF);

      $setup (posedge SCL, negedge SDA &&& TimingCheckEnable, tSU_STA);
      $setup (SDA, posedge SCL &&& TimingCheckEnable, tSU_DAT);
      $setup (posedge SCL, posedge SDA &&& TimingCheckEnable, tSU_STO);

      $hold  (negedge SDA &&& TimingCheckEnable, negedge SCL, tHD_STA);
   endspecify

endmodule
