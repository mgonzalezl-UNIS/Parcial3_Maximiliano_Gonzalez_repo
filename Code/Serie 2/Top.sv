`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 09/17/2025 07:12:11 PM
// Design Name: 
// Module Name: Top
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module Top(
    input logic clk,
    input logic reset, 
    input logic agua, cafe, leche, quiereLeche,
    input logic T, 
    output logic [2:0] TH_M);
    
    //Conexion entre FSM1 y FSM2
    logic M1, M0;
    
    typedef enum logic [2:0] {STANDBY, WORKING, POURINGCOFFEE, 
                              POURINGMILK, NEEDMILK, DONE, 
                              ENJOY} outtype;
    outtype TH;
    
    //FSM1: Genera outputs M1/M0
    FSM1 OrdenIngredientes (
        .clk(clk),
        .r(reset),
        .agua(agua), .cafe(cafe), .leche(leche), .quiereLeche(quiereLeche),
        .M0(M0), .M1(M1)
    );
    
    FSM2 PrepPour (
        .CLK(clk),
        .R(reset),
        .M1(M1), .M0(M0), .T(T),
        .TH_M(TH_M)
    );
    
    always_comb begin
        case(TH_M)
            3'b000: TH = STANDBY; 
            3'b001: TH = WORKING; 
            3'b010: TH = POURINGCOFFEE;
            3'b011: TH = POURINGMILK; 
            3'b100: TH = NEEDMILK; 
            3'b101: TH = DONE;
            3'b110: TH = ENJOY;
            default: TH = STANDBY;
         endcase
     end 
    
endmodule