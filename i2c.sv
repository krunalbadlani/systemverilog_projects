odule eeprom_top
  (
 input clk,
 input rst,
 input newd,
 input ack,
 input wr,   
 output scl,
 inout sda,
 input [7:0] wdata,
 input [6:0] addr, /////8-bit  7-bit : addr 1-bit : mode
 output reg [7:0] rdata,
 output reg done
  );
  
 reg sda_en = 0;
  
 reg sclt, sdat, donet; 
 reg [7:0] rdatat; 
 reg [7:0] addrt;
 
  
  typedef enum {new_data = 0, check_wr = 1, wstart = 2, wsend_addr = 3, waddr_ack = 4, 
                wsend_data = 5, wdata_ack = 6, wstop = 7, rsend_addr = 8, raddr_ack = 9, rsend_data = 10,
                rdata_ack = 11, rstop = 12 } state_type;
  state_type state;
  
  
  
  
  
  reg sclk_ref = 0;
  int count = 0;
  int i = 0;
  
  ////100 M / 400 K = N
  ///N/2
  
  
  always@(posedge clk)
    begin
      if(count <= 9) 
        begin
           count <= count + 1;     
        end
      else
         begin
           count     <= 0; 
           sclk_ref  <= ~sclk_ref;
         end	      
    end
  
  
  
  always@(posedge sclk_ref, posedge rst)
    begin 
      if(rst == 1'b1)
         begin
           sclt  <= 1'b0;
           sdat  <= 1'b0;
           donet <= 1'b0;
         end
       else begin
         case(state)
           new_data : 
           begin
              sdat <= 1'b0;
              done <= 1'b0;
              sda_en  <= 1'b1;
              sclt <= 1'b1;
              sdat <= 1'b1;
             if(newd == 1'b1) 
                state  <= wstart;
             else 
                 state <= new_data;         
           end
         
            wstart: 
            begin
              sdat  <= 1'b0;
              sclt  <= 1'b1;
              state <= check_wr;
              addrt <= {addr,wr};
            end
            
            
            
            check_wr: begin
                ///addr remain same for both write and read
              if(wr == 1'b1) 
                 begin
                 state <= wsend_addr;
                 sdat <= addrt[0];
                 i <= 1;
                 end
               else 
                 begin
                 state <= rsend_addr;
                 sdat <= addrt[0];
                 i <= 1;
                 end
            end
         
                    
 
 
         
         
            wsend_addr : begin                
                      if(i <= 7) begin
                      sdat  <= addrt[i];
                      i <= i + 1;
                      end
                      else
                        begin
                          i <= 0;
                          state <= waddr_ack; 
                        end   
                    end
         
         
           waddr_ack : begin
             if(ack == 1'b1) begin
               state <= wsend_data;
               sdat  <= wdata[0];
               i <= i + 1;
               end
             else
               state <= waddr_ack;
           end
         
         wsend_data : begin
           if(i <= 7) begin
              i     <= i + 1;
              sdat  <= wdata[i]; 
           end
           else begin
              i     <= 0;
              state <= wdata_ack;
           end
         end
         
          wdata_ack : begin
             if(ack == 1'b1) begin
               state <= wstop;
               sdat <= 1'b0;
               sclt <= 1'b1;
               end
             else begin
               state <= wdata_ack;
             end 
            end
         
              
         
         wstop: begin
              sdat  <=  1'b1;
              state <=  new_data;
              done  <=  1'b1;  
         end
         
         ///////////////////////read state
         
         
          rsend_addr : begin
                     if(i <= 7) begin
                      sdat  <= addrt[i];
                      i <= i + 1;
                      end
                      else
                        begin
                          i <= 0;
                          state <= raddr_ack; 
                        end   
                    end
         
         
           raddr_ack : begin
             if(ack == 1'b1) begin
               state  <= rsend_data;
               sda_en <= 1'b0;
             end
             else
               state <= raddr_ack;
           end
         
         rsend_data : begin
                   if(i <= 7) begin
                         i <= i + 1;
                         state <= rsend_data;
                         rdata[i] <= sda;
                      end
                      else
                        begin
                          i <= 0;
                          state <= rstop;
                          sclt <= 1'b1;
                          sdat <= 1'b0;  
                        end         
         end
          
        
         
         
         rstop: begin
              sdat  <=  1'b1;
              state <=  new_data;
              done  <=  1'b1;  
              end
         
         
         default : state <= new_data;
         
          	 endcase
          end
  end
  
 assign scl = (( state == wstart) || ( state == wstop) || ( state == rstop)) ? sclt : sclk_ref;
 assign sda = (sda_en == 1'b1) ? sdat : 1'bz;
endmodule
 
 
 
/////////////////////////////module i2c_memory
 
module i2cmem_top (
input clk, rst,
input scl,
inout sda,
output reg ack
);
 
reg [7:0] mem[128];
reg [7:0] addrin;
reg [7:0] datain;
reg [7:0] temprd;
reg sda_en = 0;
reg sdar = 0;
 
int i = 0;
int count = 0;
 
reg sclk_ref = 0;
  
  always@(posedge clk)
    begin
      if(count <= 9) 
        begin
           count <= count + 1;     
        end
      else
         begin
           count     <= 0; 
           sclk_ref  <= ~sclk_ref;
         end	      
    end
    
    
 
typedef enum {start = 0, store_addr = 1, ack_addr = 2, store_data = 3, ack_data  = 4, stop = 5, send_data = 6} state_type;
state_type state;
 
 
always@(posedge sclk_ref, posedge rst)
    begin 
      if(rst == 1'b1) begin
        for(int j =0; j < 127 ; j++) begin 
         mem[j] <= 8'h91;
        end
        sda_en <= 1'b1;        
       end 
      else begin
       case(state)
         start: begin
           sda_en <= 1'b1;  ///read data
          if ((scl == 1'b1) && (sda == 1'b0)) begin
               state <= store_addr;
               end
           else
              state <= start;         
         end
         
         store_addr: begin
           sda_en <= 1'b1; ///read data
            if(i <= 7) begin
            i <= i + 1;
            addrin[i] <= sda;
          end
          else begin
             state <= ack_addr;
             temprd <= mem[addrin[7:1]]; 
             ack <= 1'b1; 
             i <= 0;
             end
         end
         
         ack_addr: begin
           ack <= 1'b0;
                  
          if(addrin[0] == 1'b1) begin
             state <= store_data;
             sda_en <= 1'b1; 
             end
          else begin
             state <= send_data;
              i <= 1;
              sda_en <= 1'b0;  ///write data
              sdar <= temprd[0];
            end
         end
         
         store_data : 
         begin
         
         if(i <= 7) begin
            i <= i + 1;
            datain[i] <= sda;
          end
          else begin
             state <= ack_data; 
             ack <= 1'b1;
             i <= 0;
             end   
         end
         
         ack_data : begin
           ack <= 1'b0;
           mem[addrin[7:1]] <= datain;
           state <= stop;    
         end
         
         stop: begin
           sda_en <= 1'b1;
          if( (scl == 1'b1)&&(sda == 1'b1) )
           state <= start;
           else
           state <= stop; 
         end
         
         send_data : begin
           sda_en <= 1'b0;
          if(i <= 7) begin
            i <= i + 1;
            sdar <= temprd[i];
            end
          else begin
             state <= stop; 
             i <= 0;
             sda_en <= 1'b1;
             end 
         end
         
         default : state <= start;
       endcase
      end
    end
    
assign sda = (sda_en == 1'b1) ? 1'bz : sdar;
 
endmodule
 
 
 
///////////////////////////////////////
 
module i2c_top(
 input clk,
 input rst,
 input newd,
 input wr,   
 input [7:0] wdata,
 input [6:0] addr,
 output [7:0] rdata,
 output  done
);
 
wire sdac;
wire sclc;
wire ackc;
 
eeprom_top e1 (clk,rst,newd, ackc,wr,sclc, sdac, wdata, addr, rdata,done);
 
i2cmem_top m1 (clk,rst, sclc, sdac, ackc);
 
endmodule
 
//////////////////////////////////////////////////
 
 
interface i2c_if;
  logic clk;
  logic rst;
  logic newd;
  logic wr;   
  logic [7:0] wdata;
  logic [6:0] addr;
  logic [7:0] rdata;
  logic  done;
  logic sclk_ref;
  
  
endinterface
 


Testbench Code:



/////////////////////////Transaction
class transaction;
  
  bit newd;
  rand bit wr;
  rand bit [7:0] wdata;
  rand bit [6:0] addr;
  bit [7:0] rdata;
  bit done;
  
  constraint addr_c { addr > 0; addr < 5; }
  
  constraint rd_wr_c {
    wr dist {1 :/ 50 ,  0 :/ 50};
  }
  
  function void display( input string tag);
    $display("[%0s] : WR : %0b WDATA  : %0d ADDR : %0d RDATA : %0d DONE : %0b ",tag, wr, wdata, addr, rdata, done);
  endfunction
  
  function transaction copy();
    copy = new();
    copy.newd  = this.newd;
    copy.wr    = this.wr;
    copy.wdata = this.wdata;
    copy.addr  = this.addr;
    copy.rdata = this.rdata;
    copy.done  = this.done;
  endfunction
  
endclass
 
/////////////////////////Generator
 
class generator;
  
  transaction tr;
  mailbox #(transaction) mbxgd;
  event done; ///gen completed sending requested no. of transaction
  event drvnext; /// dr complete its wor;
  event sconext; ///scoreboard complete its work
 
   int count = 0;
  
  function new( mailbox #(transaction) mbxgd);
    this.mbxgd = mbxgd;   
    tr =new();
  endfunction
  
    task run();
    
    repeat(count) begin
      assert(tr.randomize) else $error("Randomization Failed");
      mbxgd.put(tr.copy);
      tr.display("GEN");
      @(drvnext);
      @(sconext);
    end
    -> done;
  endtask
  
   
endclass
 
 
///////////////////////////////Driver
 
 
class driver;
  
  virtual i2c_if vif;
  
  transaction tr;
  
  event drvnext;
  
  mailbox #(transaction) mbxgd;
 
  
  function new( mailbox #(transaction) mbxgd );
    this.mbxgd = mbxgd; 
  endfunction
  
  //////////////////Resetting System
  task reset();
    vif.rst <= 1'b1;
    vif.newd <= 1'b0;
    vif.wr <= 1'b0;
    vif.wdata <= 0;
    vif.addr  <= 0;
    repeat(10) @(posedge vif.clk);
    vif.rst <= 1'b0;
    repeat(5) @(posedge vif.clk);
    $display("[DRV] : RESET DONE"); 
  endtask
  
  
  task run();
    
    forever begin
      
      mbxgd.get(tr);
      
      
      @(posedge vif.sclk_ref);
      vif.rst   <= 1'b0;
      vif.newd  <= 1'b1;
      vif.wr    <= tr.wr;
      vif.wdata <= tr.wdata;
      vif.addr  <= tr.addr; 
      
      
      @(posedge vif.sclk_ref);
      vif.newd <= 1'b0;
      
      
      wait(vif.done == 1'b1);
      @(posedge vif.sclk_ref);
      wait(vif.done == 1'b0);
      
      
      $display("[DRV] : wr:%0b wdata :%0d waddr : %0d rdata : %0d", vif.wr, vif.wdata, vif.addr, vif.rdata); 
      
      ->drvnext;
    end
  endtask
  
    
  
endclass
 
 
 
/////////////////////////////////////
 
 
/* module tb;
   
  generator gen;
  driver drv;
  event next;
  event done;
  
  mailbox #(transaction) mbxgd;
  
  i2c_if vif();
  i2c_top dut (vif.clk, vif.rst,  vif.newd, vif.wr, vif.wdata, vif.addr, vif.rdata, vif.done);
 
  initial begin
    vif.clk <= 0;
  end
  
  always #5 vif.clk <= ~vif.clk;
  
  initial begin
 
    mbxgd = new();
    gen = new(mbxgd);
    drv = new(mbxgd);
    gen.count = 10;
    drv.vif = vif;
    
    drv.drvnext = next;
    gen.drvnext = next;
    
  end
  
  initial begin
    fork
      drv.reset();
      gen.run();
      drv.run();
    join_none  
    wait(gen.done.triggered);
    $finish();
  end
   
  initial begin
    $dumpfile("dump.vcd");
    $dumpvars;   
  end
 
   assign vif.sclk_ref = dut.e1.sclk_ref;
   
  
endmodule
 
*/
 
//////////////////////////////////////monitor
 
 
class monitor;
    
  virtual i2c_if vif;
  
  transaction tr;
  
  mailbox #(transaction) mbxms;
 
 
  
 
  
  function new( mailbox #(transaction) mbxms );
    this.mbxms = mbxms;
  endfunction
  
  
  task run();
    
    tr = new();
    
    forever begin
      
    @(posedge vif.sclk_ref);
      
    if(vif.newd == 1'b1) begin 
        
           if(vif.wr == 1'b0)
       		    begin
                tr.wr = vif.wr;
                tr.wdata = vif.wdata;
                tr.addr = vif.addr;
                @(posedge vif.sclk_ref);
                  
                wait(vif.done == 1'b1);
                tr.rdata = vif.rdata;
                  
                 repeat(2) @(posedge vif.sclk_ref);
                  
                $display("[MON] : DATA READ -> waddr : %0d rdata : %0d", tr.addr, tr.rdata);
               end
        
           else
           
              begin
              tr.wr = vif.wr;
              tr.wdata = vif.wdata;
              tr.addr = vif.addr;
                
              @(posedge vif.sclk_ref);
                
              wait(vif.done == 1'b1);
                
              tr.rdata = vif.rdata; 
                
              repeat(2) @(posedge vif.sclk_ref); 
                
               $display("[MON] : DATA WRITE -> wdata :%0d waddr : %0d",  tr.wdata, tr.addr);      
              end
          
           
             mbxms.put(tr);  
        
        end
 
    end
   
    
    
  endtask
  
endclass
///////////////////////////////////////////////////////////////
 
class scoreboard;
  
  transaction tr;
  
  mailbox #(transaction) mbxms;
  
  event sconext;
  
  bit [7:0] temp;
  
  bit [7:0] data[128] = '{default:0};
  
 
  
  function new( mailbox #(transaction) mbxms );
    this.mbxms = mbxms;
  endfunction
  
  
  task run();
    
    forever begin
      
      mbxms.get(tr);
      
      tr.display("SCO");
      
       if(tr.wr == 1'b1)
        begin
          
          data[tr.addr] = tr.wdata;
          
          $display("[SCO]: DATA STORED -> ADDR : %0d DATA : %0d", tr.addr, tr.wdata);
        end
       else 
        begin
         temp = data[tr.addr];
          
          if( (tr.rdata == temp) || (tr.rdata == 145) )
            $display("[SCO] :DATA READ -> Data Matched");
         else
            $display("[SCO] :DATA READ -> DATA MISMATCHED");
       end
      
        
      ->sconext;
    end 
  endtask
  
  
endclass
 
 
 
 
 
module tb;
   
  generator gen;
  driver drv;
  monitor mon;
  scoreboard sco;
  
  
  event nextgd;
  event nextgs;
 
  
  mailbox #(transaction) mbxgd, mbxms;
 
  
  i2c_if vif();
  
  i2c_top dut (vif.clk, vif.rst,  vif.newd, vif.wr, vif.wdata, vif.addr, vif.rdata, vif.done);
 
  initial begin
    vif.clk <= 0;
  end
  
  always #5 vif.clk <= ~vif.clk;
  
   initial begin
   
     
    mbxgd = new();
    mbxms = new();
    
    gen = new(mbxgd);
    drv = new(mbxgd);
    
    mon = new(mbxms);
    sco = new(mbxms);
 
    gen.count = 20;
  
    drv.vif = vif;
    mon.vif = vif;
    
    gen.drvnext = nextgd;
    drv.drvnext = nextgd;
    
    gen.sconext = nextgs;
    sco.sconext = nextgs;
  
   end
  
  task pre_test;
  drv.reset();
  endtask
  
  task test;
    fork
      gen.run();
      drv.run();
      mon.run();
      sco.run();
    join_any  
  endtask
  
  
  task post_test;
    wait(gen.done.triggered);
    $finish();    
  endtask
  
  task run();
    pre_test;
    test;
    post_test;
  endtask
  
  initial begin
    run();
  end
   
  initial begin
    $dumpfile("dump.vcd");
    $dumpvars(1,tb);   
  end
 
assign vif.sclk_ref = dut.e1.sclk_ref;   
  
endmodule
 
 


