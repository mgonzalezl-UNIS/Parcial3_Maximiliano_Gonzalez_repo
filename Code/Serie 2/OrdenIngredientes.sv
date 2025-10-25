`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 09/16/2025 11:15:24 PM
// Design Name: 
// Module Name: Procesamiento
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

module FSM1(input logic clk,
                     input logic r,
                     input logic agua, cafe, leche, quiereLeche, 
                     output logic M0, M1);
    
    //Variable sets
    typedef enum logic {S0, S1} stateType;
    stateType state, nextState;

    //State Register
    always_ff @ (posedge clk) begin 
        if(r) 
            state <= S0;
        else 
            state <= nextState;
    end 
        
    //Next State Logic
    always_comb begin
        nextState = state; 
        case(state)
            S0: begin
                if ((agua & cafe & leche & quiereLeche)||
                    (agua & cafe & ~quiereLeche) ||
                    (agua & cafe & ~leche & ~quiereLeche))
                    nextState = S1;
                else
                    nextState = S0;
                end 
                    
            S1: begin 
                if ((agua & cafe & leche & quiereLeche)||
                    (agua & cafe & ~quiereLeche) ||
                    (agua & cafe & ~leche & ~quiereLeche))
                    nextState = S1;
                else
                    nextState = S0;
                end 
                
            default: nextState = S0;
        endcase
    end 
    
    //Output Logic
    /* ----------------------------
     Output logic (Mealy - solo depende de los inputs)
     Decodificador:
       N       : M1,M0 = 0,0
       ACX~Q   : M1,M0 = 0,1  (A & C & ~Q)
       AC~LQ   : M1,M0 = 1,0  (A & C & ~L & Q)
       ACLQ    : M1,M0 = 1,1  (A & C &  L & Q)
     ----------------------------*/
    always_comb begin 
        M1 = 1'b0;
        M0 = 1'b0;

        if (agua && cafe) begin
            if (~quiereLeche) begin
                // ACX~Q
                M1 = 1'b0; M0 = 1'b1;
            end
            else if (~leche & quiereLeche) begin
                // AC~LQ
                M1 = 1'b1; M0 = 1'b0;
            end
            else if (leche & quiereLeche) begin
                // ACLQ
                M1 = 1'b1; M0 = 1'b1;
            end
        end
    end
     
endmodule