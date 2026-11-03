/*
 * M24LC64.v
 * I2C EEPROM仿真模型，用于验证I2C控制器。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
// **                   This information is distributed under license from Young Engineering.           **
// **                              COPYRIGHT (c) 2009 YOUNG ENGINEERING                                 **

`timescale 1ns/10ps

module M24LC64 (A0, A1, A2, WP, SDA, SCL, RESET);

   input                A0;                              
   input                A1;                              
   input                A2;                              

   input                WP;                              

   inout                SDA;                             
   input                SCL;                             

   input                RESET;                           

   reg                  SDA_DO;                          
   reg                  SDA_OE;                          

   wire                 SDA_DriveEnable;                 
   reg                  SDA_DriveEnableDlyd;             

   wire [02:00]         ChipAddress;                     

   reg  [03:00]         BitCounter;                      

   reg                  START_Rcvd;                      
   reg                  STOP_Rcvd;                       
   reg                  CTRL_Rcvd;                       
   reg                  ADHI_Rcvd;                       
   reg                  ADLO_Rcvd;                       
   reg                  MACK_Rcvd;                       

   reg                  WrCycle;                         
   reg                  RdCycle;                         

   reg  [07:00]         ShiftRegister;                   

   reg  [07:00]         ControlByte;                     
   wire                 RdWrBit;                         

   reg  [12:00]         StartAddress;                    
   reg  [04:00]         PageAddress;                     

   reg  [07:00]         WrDataByte [0:31];               
   wire [07:00]         RdDataByte;                      

   reg  [15:00]         WrCounter;                       

   reg  [04:00]         WrPointer;                       
   reg  [12:00]         RdPointer;                       

   reg                  WriteActive;                     

   reg  [07:00]         MemoryBlock [0:8191];            

   integer              LoopIndex;                       

   integer              tAA;                             
   integer              tWC;                             

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
      ADHI_Rcvd  = 0;
      ADLO_Rcvd  = 0;
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
         ADHI_Rcvd  <= 0;
         ADLO_Rcvd  <= 0;
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
         ADHI_Rcvd  <= 0;
         ADLO_Rcvd  <= 0;
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

   assign RdWrBit = ControlByte[00];

   always @(negedge SCL) begin
      if (CTRL_Rcvd & (BitCounter == 8)) begin
         if (RdWrBit == 0) begin
            StartAddress[12:08] <= ShiftRegister[04:00];
            RdPointer[12:08]    <= ShiftRegister[04:00];

            ADHI_Rcvd <= 1;
         end

         WrCounter <= 0;
         WrPointer <= 0;

         CTRL_Rcvd <= 0;
      end
   end

   always @(negedge SCL) begin
      if (ADHI_Rcvd & (BitCounter == 8)) begin
         if (RdWrBit == 0) begin
            StartAddress[07:00] <= ShiftRegister[07:00];
            RdPointer[07:00]    <= ShiftRegister[07:00];

            ADLO_Rcvd <= 1;
         end

         WrCounter <= 0;
         WrPointer <= 0;

         ADHI_Rcvd <= 0;
      end
   end

   always @(negedge SCL) begin
      if (ADLO_Rcvd & (BitCounter == 8)) begin
         if (RdWrBit == 0) begin
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
         PageAddress = StartAddress[04:00] + LoopIndex;

         MemoryBlock[{StartAddress[12:05],PageAddress[04:00]}] = WrDataByte[LoopIndex[04:00]];
      end
   end

   always @(negedge SCL) begin
      if (BitCounter == 8) begin
         if (WrCycle & ADLO_Rcvd) begin
            RdPointer <= StartAddress + WrPointer + 1;
         end
         if (RdCycle) begin
            RdPointer <= RdPointer + 1;
         end
      end
   end

   assign RdDataByte = MemoryBlock[RdPointer[12:00]];

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

   wire [07:00] MemoryByte_000 = MemoryBlock[00];
   wire [07:00] MemoryByte_001 = MemoryBlock[01];
   wire [07:00] MemoryByte_002 = MemoryBlock[02];
   wire [07:00] MemoryByte_003 = MemoryBlock[03];
   wire [07:00] MemoryByte_004 = MemoryBlock[04];
   wire [07:00] MemoryByte_005 = MemoryBlock[05];
   wire [07:00] MemoryByte_006 = MemoryBlock[06];
   wire [07:00] MemoryByte_007 = MemoryBlock[07];
   wire [07:00] MemoryByte_008 = MemoryBlock[08];
   wire [07:00] MemoryByte_009 = MemoryBlock[09];
   wire [07:00] MemoryByte_00A = MemoryBlock[10];
   wire [07:00] MemoryByte_00B = MemoryBlock[11];
   wire [07:00] MemoryByte_00C = MemoryBlock[12];
   wire [07:00] MemoryByte_00D = MemoryBlock[13];
   wire [07:00] MemoryByte_00E = MemoryBlock[14];
   wire [07:00] MemoryByte_00F = MemoryBlock[15];

   wire [07:00] WriteData_00 = WrDataByte[00];
   wire [07:00] WriteData_01 = WrDataByte[01];
   wire [07:00] WriteData_02 = WrDataByte[02];
   wire [07:00] WriteData_03 = WrDataByte[03];
   wire [07:00] WriteData_04 = WrDataByte[04];
   wire [07:00] WriteData_05 = WrDataByte[05];
   wire [07:00] WriteData_06 = WrDataByte[06];
   wire [07:00] WriteData_07 = WrDataByte[07];
   wire [07:00] WriteData_08 = WrDataByte[08];
   wire [07:00] WriteData_09 = WrDataByte[09];
   wire [07:00] WriteData_0A = WrDataByte[10];
   wire [07:00] WriteData_0B = WrDataByte[11];
   wire [07:00] WriteData_0C = WrDataByte[12];
   wire [07:00] WriteData_0D = WrDataByte[13];
   wire [07:00] WriteData_0E = WrDataByte[14];
   wire [07:00] WriteData_0F = WrDataByte[15];

   wire [07:00] WriteData_10 = WrDataByte[16];
   wire [07:00] WriteData_11 = WrDataByte[17];
   wire [07:00] WriteData_12 = WrDataByte[18];
   wire [07:00] WriteData_13 = WrDataByte[19];
   wire [07:00] WriteData_14 = WrDataByte[20];
   wire [07:00] WriteData_15 = WrDataByte[21];
   wire [07:00] WriteData_16 = WrDataByte[22];
   wire [07:00] WriteData_17 = WrDataByte[23];
   wire [07:00] WriteData_18 = WrDataByte[24];
   wire [07:00] WriteData_19 = WrDataByte[25];
   wire [07:00] WriteData_1A = WrDataByte[26];
   wire [07:00] WriteData_1B = WrDataByte[27];
   wire [07:00] WriteData_1C = WrDataByte[28];
   wire [07:00] WriteData_1D = WrDataByte[29];
   wire [07:00] WriteData_1E = WrDataByte[30];
   wire [07:00] WriteData_1F = WrDataByte[31];

   wire TimingCheckEnable = (RESET == 0) & (SDA_OE == 0);
   wire StopTimingCheckEnable = TimingCheckEnable && SCL;

   specify
      specparam
         tHI = 600,                                      
         tLO = 1300,                                     
         tSU_STA = 600,                                  
         tHD_STA = 600,                                  
         tSU_DAT = 100,                                  
         tSU_STO = 600,                                  
         tSU_WP = 600,                                   
         tHD_WP = 1300,                                  
         tBUF = 1300;                                    

      $width (posedge SCL, tHI);
      $width (negedge SCL, tLO);

      $width (posedge SDA &&& SCL, tBUF);

      $setup (posedge SCL, negedge SDA &&& TimingCheckEnable, tSU_STA);
      $setup (SDA, posedge SCL &&& TimingCheckEnable, tSU_DAT);
      $setup (posedge SCL, posedge SDA &&& TimingCheckEnable, tSU_STO);
      $setup (WP, posedge SDA &&& StopTimingCheckEnable, tSU_WP);

      $hold  (negedge SDA &&& TimingCheckEnable, negedge SCL, tHD_STA);
      $hold  (posedge SDA &&& StopTimingCheckEnable, WP, tHD_WP);
   endspecify

endmodule
