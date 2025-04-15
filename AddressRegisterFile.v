
//sıkıntılı
    `timescale 1ns / 1ps
    module AddressRegisterFile(
            input wire Clock,
            input wire [31:0] I,        
            input wire [2:0] RegSel,     
            input wire [1:0] FunSel,    
            input wire [1:0] OutCSel,   
            input wire [1:0] OutDSel,   
            output wire [15:0] OutC,    
            output wire [15:0] OutD         
        );

        // R[0]-R[2] : AR, SP, PC
        
        wire [15:0] AR_Out;
        wire [15:0] SP_Out;
        wire [15:0] PC_Out;

        Register16bit AR(
            .I(I[15:0]),
            .E(RegSel[0]),
            .FunSel(FunSel),
            .Clock(Clock),
            .Q(AR_Out)
        );

        Register16bit SP(
            .I(I[15:0]),
            .E(RegSel[1]),
            .FunSel(FunSel),
            .Clock(Clock),
            .Q(SP_Out)
        );

        Register16bit PC(
            .I(I[15:0]),
            .E(RegSel[2]),
            .FunSel(FunSel),
            .Clock(Clock),
            .Q(PC_Out)
        );

        assign OutC = (OutCSel == 2'b00) ? PC_Out :
                    (OutCSel == 2'b01) ? SP_Out :
                    (OutCSel == 2'b0)  ?    AR_Out:
                    (OutCSel == 2'b1)  ?    AR_Out:
                    AR_Out;

        assign OutD = (OutDSel == 2'b00) ? PC_Out :
                    (OutDSel == 2'b01) ? SP_Out :
                     (OutDSel == 2'b0)  ?AR_Out:
                    (OutDSel == 2'b1)  ? AR_Out:
                    AR_Out;

    endmodule