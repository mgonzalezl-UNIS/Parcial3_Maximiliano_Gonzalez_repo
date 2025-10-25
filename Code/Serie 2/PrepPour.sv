`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 09/17/2025 08:50:42 AM
// Design Name: 
// Module Name: FSM2
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


module FSM2 (
    input  logic CLK,
    input  logic R,    
    input  logic M1, M0, T,
    output logic [2:0] TH_M,
    output logic E2, E1, E0);    

    // Variable sets
    typedef enum logic [2:0] {S0, S1, S2, S3, S4, S5, S6} stateType;
    stateType state, nextState;
    
    typedef enum logic [2:0] {STANDBY, WORKING, POURINGCOFFEE, 
                              POURINGMILK, NEEDMILK, DONE, 
                              ENJOY} outtype;
    outtype TH;

    // State Register
    always_ff @(posedge CLK or posedge R) begin
        if (R) 
            state <= S0;             
        else   
            state <= nextState;
    end

    // Next State Logic
    always_comb begin
        nextState = state;
        case(state)
        
            S0: begin
                if ((~M1 & M0) | (M1 & M0)) 
                    nextState = S1;
                else if (M1 & ~M0) 
                    nextState = S4;
                 else
                    nextState = S0;                       
            end 
        
            S1: begin
                if (T)
                    nextState = S2;
                else if(~T)
                    nextState = S1;    
                else
                    nextState = S0;            
            end

            S2: begin
                if (M1 & M0 & T)
                    nextState = S3;
                else if (T)
                    nextState = S5;
                else if(~T) 
                    nextState= S2;
                else 
                    nextState = S0;
            end
            
            S3: begin
                if(~T)
                    nextState = S3;
                else
                    nextState = S5;
                end     
            
            S4: begin
                if(M1 & M0)
                    nextState = S1;
                else if(M1 & ~M0)
                    nextState = S4;
                else 
                    nextState = S0;
            end
            
            S5: begin
                if(~T) 
                    nextState = S6;
                else
                    nextState = S5;
            end 
            
            S6: 
                nextState = S0;
                 
            default: nextState = S0;
        endcase 
    end
    
    //Output Logic Pre-Stage
    always_comb begin 
        case(state)
                S0: TH = STANDBY;
                S1: TH = WORKING;
                S2: TH = POURINGCOFFEE;
                S3: TH = POURINGMILK;
                S4: TH = NEEDMILK;
                S5: TH = DONE;
                S6: TH = ENJOY;  
                default: TH = STANDBY;    
        endcase
    end
    
    //Output Logic            
    assign TH_M = TH;
    assign {E2, E1, E0} = state;

endmodule