`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Module Name:    ddr_rw 
//////////////////////////////////////////////////////////////////////////////////
module ddr_rw #
(

   parameter C3_NUM_DQ_PINS          = 16,    
                                       // External memory data width
   parameter C3_MEM_ADDR_WIDTH       = 14,       
                                       // External memory address width
   parameter C3_MEM_BANKADDR_WIDTH   = 3        
                                       // External memory bank address width
)
    
(			  
   input clk_50M,
	input reset_n,
	output spi_clk,
	output vga_clk,

	//sd 读模块接口信号
                           
	input [7:0] mydata,                   //从flash读出的数据
   input myvalid,                        //字节有效信号
	input pic_read_done,	                 //读SD的图像数据完成信号	
	
	//VGA 模块接口信号
	input ddr_rd_cmd,
	input ddr_addr_set,                   //ddr 的地址复位
   input ddr_rden,                       //vga读ddr图像数据请求
	output reg [127:0] ddr_data,          //ddr图像数据输出	
   
	output reg lbp_finish,
	output reg lbp_wrend_flag,
   //MIG39 控制器输出状态信号
   output c3_p0_wr_underrun,          
	output c3_p0_rd_overflow,
	output c3_p0_wr_full,
	output c3_p0_cmd_full, 
   //DDR的接口信号
   inout  [C3_NUM_DQ_PINS-1:0]                      mcb3_dram_dq,     
   output [C3_MEM_ADDR_WIDTH-1:0]                   mcb3_dram_a,      
   output [C3_MEM_BANKADDR_WIDTH-1:0]               mcb3_dram_ba,
   output                                           mcb3_dram_ras_n,
   output                                           mcb3_dram_cas_n,
   output                                           mcb3_dram_we_n,
   output                                           mcb3_dram_odt,
   output                                           mcb3_dram_cke,
   output                                           mcb3_dram_reset_n,
   output                                           mcb3_dram_dm,
   inout                                            mcb3_dram_udqs,
   inout                                            mcb3_dram_udqs_n,
   inout                                            mcb3_rzq,
   inout                                            mcb3_zio,
   output                                           mcb3_dram_udm,
   input                                            c3_sys_clk,
   output                                           c3_rst0,
   inout                                            mcb3_dram_dqs,
   inout                                            mcb3_dram_dqs_n,
   output                                           mcb3_dram_ck,
   output                                           mcb3_dram_ck_n,
   output reg [5:0]                                 ddr_read_state,                         //for chipscope debug
   output reg [29:0]		                            c3_p0_cmd_byte_addr,                    //for chipscope debug
   output reg			           	                   c3_p0_cmd_en,                           //for chipscope debug
   output reg [2:0]			                         c3_p0_cmd_instr,                        //for chipscope debug
   output reg [5:0]			                         c3_p0_cmd_bl,                           //for chipscope debug
   output   				                            c3_p0_rd_en,                            //for chipscope debug
	output     [127:0]	                            c3_p0_rd_data,
	output                                        	 c3_clk0
    );


wire c3_calib_done;
wire pic_store_done; 
assign pic_store_done=(pic_read_done)&(!ddr_write_busy);

 
//寄存器用于存储从sd读出的数据
reg [127:0] sd_data_reg;
reg [127:0] ddr_wdata_reg;

reg [9:0] counter;
reg [7:0] counter1;

reg ddr_write_busy;
reg ddr_rd_busy;

reg ddr_wren;
reg ddr_wr_req;
reg ddr_wren_reg1;
reg ddr_wren_reg2;

reg ddr_rden_req;
reg ddr_rden_reg1;
reg ddr_rden_reg2;

reg ddr_rd_cmd_req;
reg ddr_rd_cmd_reg1;
reg ddr_rd_cmd_reg2;

reg first_read;

//DDR读写的状态寄存器
reg [5:0] ddr_write_state;


parameter write_idle=6'b000000;
parameter write_fifo=6'b000001;
parameter write_data_done=6'b000010;
parameter write_cmd_start=6'b000011;
parameter write_cmd=6'b000100;
parameter write_done=6'b000101;

parameter read_idle=6'b000110;
parameter read_cmd_start=6'b000111;
parameter read_cmd=6'b001000;
parameter read_wait=6'b001001;
parameter read_data=6'b001010;
parameter read_done=6'b001011;

parameter write_lbp_idle=6'b100000;
parameter write_lbp_fifo=6'b100001;
parameter write_lbp_done=6'b100010;
parameter write_lbpcmd_start=6'b100011;
parameter write_lbpcmd=6'b100100;
parameter write_lbpcmd_done=6'b100101;
parameter write_lbp_stay=6'b101101;

parameter read_lbp_idle=6'b100110;
parameter read_lbpcmd_start=6'b100111;
parameter read_lbpcmd=6'b101000;
parameter read_lbp_wait=6'b101001;
parameter read_lbp_data=6'b101010;
parameter read_lbp_done=6'b101011;
parameter read_lbp_stay=6'b101100;

//ddr user interface

wire				c3_p0_cmd_empty;
wire				c3_p0_cmd_full;

reg				c3_p0_wr_en;
reg[15:0]	   c3_p0_wr_mask;
reg[127:0]	   c3_p0_wr_data;
wire				c3_p0_wr_full;
wire				c3_p0_wr_empty;
wire[6:0]		c3_p0_wr_count;
wire				c3_p0_wr_underrun;
wire				c3_p0_wr_error;



wire				c3_p0_rd_full;
wire				c3_p0_rd_empty;
wire[6:0]		c3_p0_rd_count;
wire				c3_p0_rd_overflow;
wire				c3_p0_rd_error;

reg            c3_p0_lbprd_en;
/*****************************************************************************/
////////////////////////LBP
/*****************************************************************************/
parameter lbp_n = 127;//2048,2048/16-1=127
parameter lbp_m = 768;//768,512
//parameter lbp_addr = 1572864;

reg [7:0] i_j;
reg [7:0] i_1_j_1;
reg [7:0] i_1_j;
reg [7:0] i_1_jadd1;
reg [7:0] i_jadd1;
reg [7:0] iadd1_jadd1;
reg [7:0] iadd1_j;
reg [7:0] iadd1_j_1;
reg [7:0] i_j_1;

reg [31:0] lbp_i;
reg [31:0] lbp_j;
reg [29:0] lbp_ddr_addr [3:0];
reg [127:0] lbp_ddr_data [3:0];
reg [127:0] lbp_ddr_datab;
reg [127:0] lbp_ddr_datap;
reg lbp_rd_on;
reg lbp_rd_off;
reg lbp_wr_on;
reg lbp_wr_off;
reg lbp_rd_flag;
reg lbp_rd_flag1;
reg lbp_rd_flag2;
reg lbp_wr_flag;
reg lbp_wr_flag1;
reg lbp_wr_flag2;
reg [7:0] lbp_code;
reg [4:0] lbp_code_data [31:0];
reg [4:0] lbp_tran_data [15:0];
reg [15:0] lbp_code_cnt;
reg [15:0] lbp_code_one;
reg lbp_wr_data_sign;
reg [7:0] lbp_cnt;
reg [127:0] lbp_wrcode_data;
reg [127:0] lbp_wr_data;
reg lbp_cal_sign;
reg [127:0] lbp_ddr_wrdata;
reg [3:0] lbp_wr_cnt;

reg lbp_rd_state;
reg lbp_wr_state;
reg lbp_wr_data_on;
reg lbp_wr_data_off;

reg [2303:0] lbp_rt_reg;
reg [31:0] lbp_cnt_reg;
reg [31:0] lbp_j_cnt;

/*****************************************************************************/
//读取16个字节(8个像素)的sd数据转换为128bit的数据存在data寄存器中
/*****************************************************************************/
reg [7:0] gray_cnt;
reg odd_flag;
reg even_flag;
reg [7:0] gray_data_reg;
reg [7:0] gray_data_ddr;
reg [4:0] gray_data_R;
reg [5:0] gray_data_G;
reg [4:0] gray_data_B;

always @(negedge spi_clk)
begin
	if(c3_rst0 || !c3_calib_done)	begin
			counter1<=10'd0;
			ddr_wren<=1'b0;
			sd_data_reg<=0;
			ddr_wdata_reg<=0;
			
			odd_flag <= 1'b1;
		   even_flag <= 1'b0;
			gray_data_reg <= 8'd0;
			gray_data_ddr <= 8'd0;
			gray_data_R <= 5'd0;
			gray_data_G <= 6'd0;
			gray_data_B <= 5'd0;
	end		
   else if (myvalid)
      begin   
	     if(counter1 <= 14 && odd_flag == 1'b1) 
		  begin                             //读取前15个sd数据	  
			    //==//sd_data_reg<={sd_data_reg[119:0],mydata};
			    gray_data_reg <= mydata;
				 counter1<=counter1+1;
				 ddr_wren<=1'b0;
				 odd_flag <= 1'b0;
				 even_flag <= 1'b1;
				 if(counter1 > 0)
				 begin
				 sd_data_reg <= {sd_data_reg[111:0],gray_data_R,gray_data_G,gray_data_B};//2{gray_data_ddr}
				 end
		  end
		  else if(counter1 <= 15 && even_flag == 1'b1)
		  begin
		       gray_data_R <= ((gray_data_reg[7:3]*156760 + {gray_data_reg[2:0],mydata[7:5]}*153876 + mydata[4:0]*59776) >> 19);
				 gray_data_G <= ((gray_data_reg[7:3]*156760 + {gray_data_reg[2:0],mydata[7:5]}*153876 + mydata[4:0]*59776) >> 18);
				 gray_data_B <= ((gray_data_reg[7:3]*156760 + {gray_data_reg[2:0],mydata[7:5]}*153876 + mydata[4:0]*59776) >> 19);
		       gray_data_ddr <= ((gray_data_reg[7:3]*156760 + {gray_data_reg[2:0],mydata[7:5]}*153876 + mydata[4:0]*59776) >> 16);
		       counter1<=counter1+1;
				 ddr_wren<=1'b0;
				 odd_flag <= 1'b1;
				 even_flag <= 1'b0;
		  end
        else if(counter1 > 15)
		  begin                                        //读取第16个sd数据
			    //==//ddr_wdata_reg<={sd_data_reg[119:0],mydata};
				 odd_flag <= 1'b0;
				 even_flag <= 1'b1;
				 gray_data_reg <= mydata;
				 ddr_wdata_reg<={sd_data_reg[111:0],gray_data_R,gray_data_G,gray_data_B};
				 sd_data_reg<=0;
				 counter1<=1;
				 ddr_wren<=1'b1;                        //接收到16个bytes数据,产生ddr写信号
		  end
		end
   else 
      ddr_wren<=1'b0;
	  
end
	  
/*****************************************************************************/
//脉宽转化,ddr_wren--->ddr_wr_req
/*****************************************************************************/
always @(negedge c3_clk0)
begin
	if(c3_rst0 || !c3_calib_done)  begin
	    ddr_wren_reg1<=1'b0;
	    ddr_wren_reg2<=1'b0;
	    ddr_wr_req<=1'b0;
	end
   else begin
	  	 ddr_wren_reg1<=ddr_wren;
	    ddr_wren_reg2<=ddr_wren_reg1;   
	    if(ddr_wren_reg1 && !ddr_wren_reg2)           //如果检测到ddr_wren的上升沿,产生ddr写请求
		   ddr_wr_req<=1'b1;
		 else
		   ddr_wr_req<=1'b0;
	end
end
 
/*****************************************************************************/
//DDR读数据请求信号脉宽处理程序: ddr_rden->ddr_rden_req
/*****************************************************************************/
always @(posedge c3_clk0)
begin
	if(c3_rst0 || !c3_calib_done) begin
     ddr_rden_reg1<=1'b0;
     ddr_rden_reg2<=1'b0; 
	  ddr_rden_req<=1'b0;	
   end
   else begin
     ddr_rden_reg1<=ddr_rden;
     ddr_rden_reg2<=ddr_rden_reg1;
     if(ddr_rden_reg1 && !ddr_rden_reg2)           //如果检测到ddr_rden的上升沿,产生ddr读数据请求
		   ddr_rden_req<=1'b1;
	  else
		   ddr_rden_req<=1'b0;	
    end
end	 

assign c3_p0_rd_en = (lbp_finish == 1'b1)?ddr_rden_req:c3_p0_lbprd_en;

/*****************************************************************************/
//DDR burst读命令请求信号脉宽处理程序: ddr_rd_cmd->ddr_rd_cmd_req
/*****************************************************************************/
always @(negedge c3_clk0)
begin
	if(c3_rst0 || !c3_calib_done) begin
     ddr_rd_cmd_reg1<=1'b0;
     ddr_rd_cmd_reg2<=1'b0; 
	  ddr_rd_cmd_req<=1'b0;	
   end
   else begin
     ddr_rd_cmd_reg1<=ddr_rd_cmd;
     ddr_rd_cmd_reg2<=ddr_rd_cmd_reg1;
     if(ddr_rd_cmd_reg1 && !ddr_rd_cmd_reg2)           //ddr_rd_cmd,产生ddr burst读命令请求
		   ddr_rd_cmd_req<=1'b1;
	  else
		   ddr_rd_cmd_req<=1'b0;	
    end
end
 
/*****************************************************************************/
//把Ram寄存器的16字节的数据写入ddr中
/*****************************************************************************/
reg [7:0] delay_cnt;
always @(posedge c3_clk0)
begin
	if(c3_rst0 || !c3_calib_done) begin  			 
     c3_p0_wr_en<=1'b0;
	  c3_p0_wr_mask<=16'd0;
	  c3_p0_wr_data<=128'd0;
	  ddr_write_busy <=1'b0;
     c3_p0_cmd_en<=1'b0;
     c3_p0_cmd_instr<=3'd0;
     c3_p0_cmd_bl<=6'd0;
     c3_p0_cmd_byte_addr<=30'd0;
     ddr_write_state<=write_idle;
     ddr_rd_busy <=1'b0;
     
     lbp_ddr_data[0] <= 128'd0;
	  lbp_ddr_data[1] <= 128'd0;
	  lbp_ddr_data[2] <= 128'd0;
	  lbp_ddr_data[3] <= 128'd0;
	  lbp_rd_off <= 1'b1;
	  lbp_cnt <= 8'd0;
	  lbp_ddr_wrdata <= 0;
	  lbp_wr_cnt <= 4'd0;
     lbp_wr_off <= 1'b1;
	  
	  c3_p0_lbprd_en <= 0;
	  lbp_wrend_flag <= 1'b0;
	  delay_cnt <= 8'd0;
    end
  else begin
  case(ddr_write_state)
		write_idle:begin			  
            c3_p0_wr_en<=1'b0;
	         c3_p0_wr_mask<=16'd0;
				if(ddr_wr_req) begin                             //如果写DDR请求信号为高				
					   ddr_write_busy<=1'b1;                //ddr写数据忙标志
						ddr_write_state<=write_fifo;
						c3_p0_wr_data<=ddr_wdata_reg;        //准备写入DDR的数据
				end
				else begin
				 if(pic_store_done) 
				 begin
				    if(lbp_finish == 1'b0)
					 begin
					    if(lbp_wr_on == 1'b1)
				       begin
						    lbp_ddr_wrdata <= lbp_wr_data;
				          lbp_wr_off <= 1'b0;
				          ddr_write_state<=write_lbp_idle;
						 end
						 else if(lbp_rd_on == 1'b1)
				       begin
				          lbp_rd_off <= 1'b0;
				          ddr_write_state<=read_lbp_idle;
				       end
					 end
					 else begin
						if(ddr_addr_set==1'b1) begin             
							c3_p0_cmd_byte_addr<=30'd16;	           //ddr的地址置位, 初始值为地址16 
						end
						else begin
							if(ddr_rd_cmd_req==1'b1) begin              //如果有ddr读命令请求
							ddr_write_state<=read_cmd_start;
							ddr_rd_busy <=1'b1;
							end
						end
					end
				 end
				end
	   end
		write_fifo:begin	  
	         if(!c3_p0_wr_full) begin                      //如p0写fifo数据不满				
						c3_p0_wr_en<=1'b1;    
				      ddr_write_state<=write_data_done;
				end			   
		end
      write_data_done:begin
			  	c3_p0_wr_en<=1'b0;
			   ddr_write_state<=write_cmd_start;
      end
		write_cmd_start:begin
            c3_p0_cmd_en<=1'b0;                    
            c3_p0_cmd_instr<=3'b010;                  //010为写命令
            c3_p0_cmd_bl<=6'd0;                       //burst length为1个128bit数据
            c3_p0_cmd_byte_addr<=c3_p0_cmd_byte_addr+16;	   //地址加16
				ddr_write_state<=write_cmd;
      end
      write_cmd:begin
			   if (!c3_p0_cmd_full) begin                        //如果命令FIFO不满				  
                 c3_p0_cmd_en<=1'b1;                   //写命令使能
				     ddr_write_state<=write_done;
				end
      end
      write_done:begin
            c3_p0_cmd_en<=1'b0;
            ddr_write_state<=write_idle;
            ddr_write_busy<=1'b0;
      end
		
		read_cmd_start:begin
				c3_p0_cmd_en<=1'b0;
				c3_p0_cmd_instr<=3'b001;               //命令字为读
				c3_p0_cmd_bl<=6'd63;                   //64个数据读
				ddr_write_state<=read_cmd; 
		end						 
		read_cmd:begin			
				c3_p0_cmd_en<=1'b1;                    //ddr读命令使能
				ddr_write_state<=read_done;
		end
		read_done:begin
				c3_p0_cmd_en<=1'b0; 
				ddr_rd_busy <=1'b0;
				c3_p0_cmd_byte_addr<=c3_p0_cmd_byte_addr+1024;    //ddr的读地址加1024 (64*128bit/8)
				first_read<=1'b0;
				ddr_write_state<=write_idle;
		end
		
      //////////////////////////////////////////////////LBP
      read_lbp_idle:begin
			   c3_p0_cmd_en<=1'b0;
            c3_p0_cmd_instr<=3'd0;
            c3_p0_cmd_bl<=6'd0;
				c3_p0_lbprd_en <= 0;
            //c3_p0_cmd_byte_addr_lbpr<=30'd16;
				///if(delay_cnt <= 5)
				///begin
				///   delay_cnt <= delay_cnt + 1;
				///end
				///else begin
				   if(lbp_cnt <= 3)
				   begin
				   c3_p0_cmd_byte_addr<=30'd16 + lbp_ddr_addr[lbp_cnt];
			      ddr_write_state<=read_lbpcmd_start;
               delay_cnt <= 0;
				   end
				   else if(lbp_cnt > 3)
				   begin
				   lbp_rd_off <= 1'b1;
				   lbp_cnt <= 0;
					ddr_write_state<=write_idle;
					delay_cnt <= 0;
				   end
				///end
	   end
		read_lbpcmd_start:begin
			   c3_p0_cmd_en<=1'b0;
				c3_p0_cmd_instr<=3'b001;              //命令字为读
				c3_p0_cmd_bl<=6'd0;                   //4个数据读
				ddr_write_state<=read_lbpcmd;
		end
		read_lbpcmd:begin	
            ///if (!c3_p0_cmd_full)
            ///begin		
            lbp_cnt <= lbp_cnt + 1;				
			   c3_p0_cmd_en<=1'b1;                    //ddr读命令使能
				ddr_write_state<=read_lbp_done;
				///end
		end
		/*read_lbp_stay:begin
		      c3_p0_cmd_en<=1'b0;                    //ddr读命令使能
				if(!c3_p0_rd_empty)
				begin
				ddr_write_state<=read_lbp_done;
				end
		end*/
		read_lbp_done:begin
		      c3_p0_lbprd_en <= 1;
				c3_p0_cmd_en<=1'b0;				
				//c3_p0_cmd_byte_addr_lbpr<=c3_p0_cmd_byte_addr_lbpr+1024;    //ddr的读地址加1024 (64*128bit/8)
				ddr_write_state<=read_lbp_stay;
		end
		read_lbp_stay:begin
		      c3_p0_lbprd_en <= 0;
				c3_p0_cmd_en<=1'b0;
				//if (c3_p0_rd_empty)
				//begin
				lbp_ddr_data[lbp_cnt-1] <= c3_p0_rd_data;	
				ddr_write_state<=read_lbp_idle;
				//end
		end
		//////////////////////////////////////////////////LBP
      write_lbp_idle:begin			  
            c3_p0_wr_en<=1'b0;
	         c3_p0_wr_mask<=16'd0;
				if(delay_cnt <= 2)
				begin
				   delay_cnt <= delay_cnt + 1;
			   end
				else begin
				   if(lbp_wr_cnt < 1)
				   begin
					delay_cnt <= 0;
					ddr_write_state<=write_lbp_fifo;
				   end
				   else begin
					lbp_wr_off <= 1'b1;
					lbp_wr_cnt <= 0;
					delay_cnt <= 0;
					ddr_write_state<=write_idle;
				   end
				end
	   end
		write_lbp_fifo:begin	  
	         if(!c3_p0_wr_full) begin                      //如p0写fifo数据不满				
						c3_p0_wr_en<=1'b1;    
						c3_p0_wr_data<=lbp_ddr_wrdata;        //准备写入DDR的数据
						c3_p0_cmd_instr<=3'b010;                  //010为写命令
                  c3_p0_cmd_bl<=6'd0;                       //burst length为1个128bit数据
                  c3_p0_cmd_byte_addr<= 16 + lbp_ddr_addr[2];	   //地址加16	
				      ddr_write_state<=write_lbpcmd;
				end			   
		end
      /*write_lbp_done:begin
			  	c3_p0_wr_en<=1'b0;			
			   ddr_write_state<=write_lbpcmd_start;
      end
		write_lbpcmd_start:begin
            c3_p0_cmd_en<=1'b0;                    
            c3_p0_cmd_instr<=3'b010;                  //010为写命令
            c3_p0_cmd_bl<=6'd0;                       //burst length为1个128bit数据
            c3_p0_cmd_byte_addr<=lbp_ddr_addr[lbp_wr_cnt+2];	   //地址加16
				ddr_write_state<=write_lbpcmd;
				
      end*/
      write_lbpcmd:begin
			   ///if (!c3_p0_cmd_full) begin                        //如果命令FIFO不满 
            c3_p0_wr_en<=1'b0;                           			
            if (!c3_p0_cmd_full)
				begin                        //如果命令FIFO不满  	                    					
			      c3_p0_cmd_en<=1'b1;	
				   ddr_write_state<=write_lbp_done;
				end
				///end
      end
      write_lbp_done:begin
            c3_p0_cmd_en<=1'b0;
				lbp_wrend_flag <= (lbp_i == 1)?1'b1:1'b0;
            ddr_write_state<=write_lbp_idle;            
				lbp_wr_cnt <= lbp_wr_cnt + 1;
      end
		
      //////////////////////////////////////////////////LBP		
		default:begin		
		      c3_p0_wr_en<=1'b0;
            c3_p0_cmd_en<=1'b0;
            c3_p0_cmd_instr<=3'd0; 
            c3_p0_cmd_bl<=6'd0;
            ddr_write_state<=write_idle;
      end				  
      endcase;			
   end
end  

/*****************************************************************************/
///////////////////////////////LBP
/*****************************************************************************/
always @(posedge c3_clk0)
begin
	if(c3_rst0 || !c3_calib_done) 
	begin
	   i_j <= 8'd0;
		i_1_j_1 <= 8'd0;
		i_1_j <= 8'd0;
		i_1_jadd1 <= 8'd0;
		i_jadd1 <= 8'd0;
		iadd1_jadd1 <= 8'd0;
		iadd1_j <= 8'd0;
		iadd1_j_1 <= 8'd0;
		i_j_1 <= 8'd0;
		
		lbp_i <= 32'd0;//768
		lbp_j <= 32'd0;//2048
		
		lbp_ddr_addr[0] <= 30'd0;
		lbp_ddr_addr[1] <= 30'd0;
		lbp_ddr_addr[2] <= 30'd0;
		lbp_ddr_addr[3] <= 30'd0;
		lbp_ddr_datab <= 128'd0;
		lbp_ddr_datap <= 128'd0;

		lbp_rd_on <= 1'b0;
		lbp_rd_flag <= 1'b0;
		lbp_rd_flag1 <= 1'b0;
		lbp_rd_flag2 <= 1'b0;
		lbp_wr_flag <= 1'b1;
		lbp_wr_flag1 <= 1'b0;
		lbp_wr_flag2 <= 1'b0;
		
		lbp_code_cnt <= 16'd0;
		lbp_wr_data_on <= 1'b0;	
		lbp_wr_data_off <= 1'b0;
		lbp_code_one <= 16'd0;
		lbp_wrcode_data <= 128'd0;
		lbp_wr_data <= 128'd0;
		lbp_cal_sign <= 1'b0;
		lbp_wr_on <= 1'b0;
		lbp_finish <= 1'b0;
		lbp_rd_state <= 1'b0;
		lbp_wr_state <= 1'b0;
		
		lbp_rt_reg <= 2304'd0;//LBP
		lbp_cnt_reg <= 32'd0;//LBP
		lbp_j_cnt <= 32'd0;//LBP
	end
	else begin
	   if(pic_store_done) 
		begin
		   if(lbp_i < lbp_m && lbp_wr_flag == 1'b1 && lbp_j < lbp_n)
			begin 
				lbp_ddr_addr[2] <= (lbp_i<<11) + (lbp_j<<4);
				lbp_ddr_addr[3] <= (lbp_i<<11) + ((lbp_j+1)<<4);
				lbp_ddr_addr[0] <= ((lbp_i+1)<<11) + (lbp_j<<4);
				lbp_ddr_addr[1] <= ((lbp_i+1)<<11) + ((lbp_j+1)<<4);
				lbp_rd_on <= 1'b1;
				lbp_rd_state <= 1'b1;
				lbp_wr_flag <= 1'b0;
				lbp_ddr_datab <= lbp_ddr_data[2];
		      lbp_ddr_datap <= lbp_ddr_data[3];
			end
			else if(lbp_i < lbp_m && lbp_wr_flag == 1'b1 && lbp_j == 127)
			begin
			   lbp_ddr_addr[2] <= (lbp_i<<11) + (lbp_j<<4);
				lbp_ddr_addr[3] <= (lbp_i<<11) + ((lbp_j)<<4);
				lbp_ddr_addr[0] <= ((lbp_i+1)<<11) + (lbp_j<<4);
				lbp_ddr_addr[1] <= ((lbp_i+1)<<11) + ((lbp_j)<<4);
				lbp_rd_on <= 1'b1;
				lbp_rd_state <= 1'b1;
				lbp_wr_flag <= 1'b0;
				lbp_ddr_datab <= lbp_ddr_data[2];
		      lbp_ddr_datap <= lbp_ddr_data[3];
			end
			else if(lbp_i >= lbp_m && lbp_wr_flag == 1'b1 && lbp_j <= lbp_n)
			begin
			   //lbp_finish <= 1'b1;
				lbp_j <= lbp_j + 1;
			   lbp_j_cnt <= 0;
				lbp_cnt_reg <= 0;
				lbp_i <= 32'd0;
				lbp_wr_flag <= 1'b1;
				
				lbp_wrcode_data <= 128'd0;
		      lbp_wr_data <= 128'd0;
				lbp_code_cnt <= 16'd0;
		      lbp_code_one <= 16'd0;
				i_j <= 8'd0;
		      i_1_j_1 <= 8'd0;
		      i_1_j <= 8'd0;
		      i_1_jadd1 <= 8'd0;
		      i_jadd1 <= 8'd0;
		      iadd1_jadd1 <= 8'd0;
		      iadd1_j <= 8'd0;
		      iadd1_j_1 <= 8'd0;
		      i_j_1 <= 8'd0;
		
		      lbp_ddr_addr[0] <= 30'd0;
		      lbp_ddr_addr[1] <= 30'd0;
		      lbp_ddr_addr[2] <= 30'd0;
		      lbp_ddr_addr[3] <= 30'd0;
		      lbp_ddr_datab <= 128'd0;
		      lbp_ddr_datap <= 128'd0;
			end
			else if(lbp_j > lbp_n)
			begin
			   lbp_finish <= 1'b1;
				
			end
			if(lbp_cnt == 1)
			begin
			   lbp_rd_on <= 1'b0;
			end
			if(lbp_wr_cnt == 1)
			begin
			   lbp_wr_on <= 1'b0;
			end
         lbp_rd_flag1 <= lbp_rd_off;
			lbp_rd_flag2 <= lbp_rd_flag1;
			if(lbp_rd_flag1 && !lbp_rd_flag2)//上升沿检测
			begin
			   
				lbp_rd_flag <= 1'b1;
				lbp_rd_state <= 1'b0;
				lbp_code_data[0] <= (lbp_ddr_data[0]>>112);
				lbp_code_data[1] <= (lbp_ddr_data[0]>>96);
				lbp_code_data[2] <= (lbp_ddr_data[0]>>80);
				lbp_code_data[3] <= (lbp_ddr_data[0]>>64);
				lbp_code_data[4] <= (lbp_ddr_data[0]>>48);
				lbp_code_data[5] <= (lbp_ddr_data[0]>>32);
				lbp_code_data[6] <= (lbp_ddr_data[0]>>16);
				lbp_code_data[7] <= lbp_ddr_data[0];
				lbp_code_data[8] <= (lbp_ddr_data[1]>>112);
				lbp_code_data[9] <= (lbp_ddr_data[1]>>96);
				lbp_code_data[10] <= (lbp_ddr_data[1]>>80);
				lbp_code_data[11] <= (lbp_ddr_data[1]>>64);
            lbp_code_data[12] <= (lbp_ddr_data[1]>>48);
				lbp_code_data[13] <= (lbp_ddr_data[1]>>32);
				lbp_code_data[14] <= (lbp_ddr_data[1]>>16);
				lbp_code_data[15] <= lbp_ddr_data[1];
				lbp_code_data[16] <= (lbp_ddr_datab>>112);
				lbp_code_data[17] <= (lbp_ddr_datab>>96);
				lbp_code_data[18] <= (lbp_ddr_datab>>80);
				lbp_code_data[19] <= (lbp_ddr_datab>>64);
				lbp_code_data[20] <= (lbp_ddr_datab>>48);
				lbp_code_data[21] <= (lbp_ddr_datab>>32);
				lbp_code_data[22] <= (lbp_ddr_datab>>16);
				lbp_code_data[23] <= lbp_ddr_datab;
				lbp_code_data[24] <= (lbp_ddr_datap>>112);
				lbp_code_data[25] <= (lbp_ddr_datap>>96);
				lbp_code_data[26] <= (lbp_ddr_datap>>80);
				lbp_code_data[27] <= (lbp_ddr_datap>>64);
            lbp_code_data[28] <= (lbp_ddr_datap>>48);
				lbp_code_data[29] <= (lbp_ddr_datap>>32);
				lbp_code_data[30] <= (lbp_ddr_datap>>16);
				lbp_code_data[31] <= lbp_ddr_datap;
				lbp_tran_data[0] <= (lbp_ddr_data[2]>>112);
				lbp_tran_data[1] <= (lbp_ddr_data[2]>>96);
				lbp_tran_data[2] <= (lbp_ddr_data[2]>>80);
				lbp_tran_data[3] <= (lbp_ddr_data[2]>>64);
				lbp_tran_data[4] <= (lbp_ddr_data[2]>>48);
				lbp_tran_data[5] <= (lbp_ddr_data[2]>>32);
				lbp_tran_data[6] <= (lbp_ddr_data[2]>>16);
				lbp_tran_data[7] <= lbp_ddr_data[2];
				lbp_tran_data[8] <= (lbp_ddr_data[3]>>112);
				lbp_tran_data[9] <= (lbp_ddr_data[3]>>96);
				lbp_tran_data[10] <= (lbp_ddr_data[3]>>80);
				lbp_tran_data[11] <= (lbp_ddr_data[3]>>64);
            lbp_tran_data[12] <= (lbp_ddr_data[3]>>48);
				lbp_tran_data[13] <= (lbp_ddr_data[3]>>32);
				lbp_tran_data[14] <= (lbp_ddr_data[3]>>16);
				lbp_tran_data[15] <= lbp_ddr_data[3];
				
			end
			if(lbp_rd_flag == 1'b1)
			begin
			   if(lbp_code_cnt <= 7)
			   begin
						if(lbp_code_cnt == 0)
						begin
							i_j <= (lbp_tran_data[0]<<3);
							i_1_j_1 <= 8'd0;
							i_1_j <= (lbp_code_data[16]<<3);
							i_1_jadd1 <= (lbp_code_data[17]<<3);
							i_jadd1 <= (lbp_tran_data[1]<<3);
							iadd1_jadd1 <= (lbp_code_data[1]<<3);
							iadd1_j <= (lbp_code_data[0]<<3);
							iadd1_j_1 <= 8'd0;
							i_j_1 <= 8'd0;
							lbp_rd_flag <= 1'b0;
							lbp_cal_sign <= 1'b1;
						end
						/*else if(lbp_code_cnt == 15)
						begin
							i_j <= (lbp_tran_data[15]<<3);
							i_1_j_1 <= (lbp_code_data[29]<<3);
							i_1_j <= (lbp_code_data[28]<<3);
							i_1_jadd1 <= 8'd0;
							i_jadd1 <= 8'd0;
							iadd1_jadd1 <= 8'd0;
							iadd1_j <= (lbp_code_data[15]<<3);
							iadd1_j_1 <= (lbp_code_data[14]<<3);
							i_j_1 <= (lbp_tran_data[14]<<3);
							lbp_rd_flag <= 1'b0;
							lbp_cal_sign <= 1'b1;
						end*/
						else begin
							i_j <= (lbp_tran_data[lbp_code_cnt]<<3);
							i_1_j_1 <= (lbp_code_data[15+lbp_code_cnt]<<3);
							i_1_j <= (lbp_code_data[16+lbp_code_cnt]<<3);
							i_1_jadd1 <= (lbp_code_data[17+lbp_code_cnt]<<3);
							i_jadd1 <= (lbp_tran_data[lbp_code_cnt+1]<<3);
							iadd1_jadd1 <= (lbp_code_data[lbp_code_cnt+1]<<3);
							iadd1_j <= (lbp_code_data[lbp_code_cnt]<<3);
							iadd1_j_1 <= (lbp_code_data[lbp_code_cnt-1]<<3);
							i_j_1 <= (lbp_tran_data[lbp_code_cnt-1]<<3);
							lbp_rd_flag <= 1'b0;
							lbp_cal_sign <= 1'b1;
						end  					
				end
				else begin
				   lbp_code_cnt <= 0;
				   lbp_wr_on <= 1'b1;
					lbp_wr_state <= 1'b1;
					lbp_wr_data <= lbp_wrcode_data;
				   lbp_rd_flag <= 1'b0;
				   lbp_cal_sign <= 1'b0;
					lbp_rt_reg[lbp_cnt_reg+2] <= (lbp_tran_data[8]>lbp_code_data[23])?1:0;
				   lbp_rt_reg[lbp_cnt_reg+1] <= (lbp_tran_data[8]>lbp_tran_data[7])?1:0;
				   lbp_rt_reg[lbp_cnt_reg] <= (lbp_tran_data[8]>lbp_code_data[7])?1:0;
               if(lbp_j > 0)
					begin
                  lbp_j_cnt <= lbp_j_cnt + 3;
					end
				end
			end
			if(lbp_cal_sign == 1'b1)
		   begin
			    if(lbp_code_cnt == 0 && lbp_j > 0)
				 begin
				 lbp_code[0] <= lbp_rt_reg[lbp_j_cnt+2];
				 lbp_code[1] <= (i_1_j > i_j)?1:0;			 
				 lbp_code[2] <= (i_1_jadd1 > i_j)?1:0;
				 lbp_code[3] <= (i_jadd1 > i_j)?1:0;
				 lbp_code[4] <= (iadd1_jadd1 > i_j)?1:0;
				 lbp_code[5] <= (iadd1_j > i_j)?1:0;
				 lbp_code[6] <= lbp_rt_reg[lbp_j_cnt];
				 lbp_code[7] <= lbp_rt_reg[lbp_j_cnt+1];
				 lbp_cal_sign <= 1'b0;
				 lbp_wr_data_on <= 1'b1;
				 lbp_code_cnt <= lbp_code_cnt + 1;	
             end
             else begin
             lbp_code[0] <= (i_1_j_1 > i_j)?1:0;
				 lbp_code[1] <= (i_1_j > i_j)?1:0;			 
				 lbp_code[2] <= (i_1_jadd1 > i_j)?1:0;
				 lbp_code[3] <= (i_jadd1 > i_j)?1:0;
				 lbp_code[4] <= (iadd1_jadd1 > i_j)?1:0;
				 lbp_code[5] <= (iadd1_j > i_j)?1:0;
				 lbp_code[6] <= (iadd1_j_1 > i_j)?1:0;
				 lbp_code[7] <= (i_j_1 > i_j)?1:0;
				 lbp_cal_sign <= 1'b0;
				 lbp_wr_data_on <= 1'b1;
				 lbp_code_cnt <= lbp_code_cnt + 1;
             end				 
		   end
			if(lbp_wr_data_on == 1'b1)
			begin
			   lbp_code_one[15:11] <= (lbp_code>>3);
				lbp_code_one[10:5] <= (lbp_code>>2);
				lbp_code_one[4:0] <= (lbp_code>>3);
				lbp_wr_data_on <= 1'b0;
				lbp_wr_data_off <= 1'b1;
			end
			if(lbp_wr_data_off == 1'b1)
			begin
			   lbp_wrcode_data <= {lbp_wrcode_data[111:0],lbp_code_one};
				lbp_wr_data_off <= 1'b0;
				lbp_rd_flag <= 1'b1;
				lbp_code <= 0;
			end
			lbp_wr_flag1 <= lbp_wr_off;
			lbp_wr_flag2 <= lbp_wr_flag1;
			if(lbp_wr_flag1 && !lbp_wr_flag2)//上升沿检测
			begin
			   lbp_wr_flag <= 1'b1;
				lbp_wr_state <= 1'b0;
				lbp_wrcode_data <= 0;
				lbp_i <= lbp_i + 1;
				lbp_cnt_reg <= lbp_cnt_reg + 3;
				
				//lbp_finish <= 1'b1;
			end
			
		end
	end
end



/*****************************************************************************/
//MIG的DDR控制器程序例化
/*****************************************************************************/
      mig_39_2 #
      (
         .C3_P0_MASK_SIZE                (16),
         .C3_P0_DATA_PORT_SIZE           (128),
         .DEBUG_EN                       (0),   //   = 0, Disable debug signals/controls.
         .C3_MEMCLK_PERIOD               (3200),
         .C3_CALIB_SOFT_IP               ("TRUE"),            // # = TRUE, Enables the soft calibration logic,
         .C3_SIMULATION                  ("FALSE"),           // # = FALSE, Implementing the design.
         .C3_RST_ACT_LOW                 (1),                 // # = 1 for active low reset         change for AX516 board
         .C3_INPUT_CLK_TYPE              ("SINGLE_ENDED"),
         .C3_MEM_ADDR_ORDER              ("ROW_BANK_COLUMN"),
         .C3_NUM_DQ_PINS                 (16),
         .C3_MEM_ADDR_WIDTH              (13),  
         .C3_MEM_BANKADDR_WIDTH          (3)
         )
      mig_37_inst
      (
         .mcb3_dram_dq			                 (mcb3_dram_dq),
         .mcb3_dram_a			                 (mcb3_dram_a), 
         .mcb3_dram_ba			                 (mcb3_dram_ba),
         .mcb3_dram_ras_n			              (mcb3_dram_ras_n),
         .mcb3_dram_cas_n			              (mcb3_dram_cas_n),
         .mcb3_dram_we_n  	                    (mcb3_dram_we_n),
         .mcb3_dram_odt			                 (mcb3_dram_odt),
         .mcb3_dram_reset_n			           (mcb3_dram_reset_n),	
         .mcb3_dram_cke                        (mcb3_dram_cke),
         .mcb3_dram_dm                         (mcb3_dram_dm),
         .mcb3_dram_udqs                       (mcb3_dram_udqs),
         .mcb3_dram_udqs_n	                    (mcb3_dram_udqs_n),
         .mcb3_rzq	                          (mcb3_rzq),
         .mcb3_zio	                          (mcb3_zio),
         .mcb3_dram_udm	                       (mcb3_dram_udm),
         .c3_sys_clk	                          (clk_50M),
         .c3_sys_rst_i	                       (reset_n),                 			
			.c3_calib_done	                       (c3_calib_done),
         .c3_clk0	                             (c3_clk0),                 //User clock
			.spi_clk	                             (spi_clk),                //AX516 board: for spi clock 
		   .vga_clk	                             (vga_clk),                 //AX516 board: for vga clock 
         .c3_rst0	                             (c3_rst0),			
			.mcb3_dram_dqs                        (mcb3_dram_dqs),
			.mcb3_dram_dqs_n	                    (mcb3_dram_dqs_n),
			.mcb3_dram_ck	                       (mcb3_dram_ck),			
			.mcb3_dram_ck_n	                    (mcb3_dram_ck_n),				
			
         // User Port-0 command interface
         .c3_p0_cmd_clk                  (c3_clk0),          //c3_p0_cmd_clk->c3_clk0			
         .c3_p0_cmd_en                   (c3_p0_cmd_en),
         .c3_p0_cmd_instr                (c3_p0_cmd_instr),
         .c3_p0_cmd_bl                   (c3_p0_cmd_bl),
         .c3_p0_cmd_byte_addr            (c3_p0_cmd_byte_addr),
         .c3_p0_cmd_empty                (c3_p0_cmd_empty),
         .c3_p0_cmd_full                 (c3_p0_cmd_full),	
			
         // User Port-0 data write interface 			
         .c3_p0_wr_clk                   (c3_clk0),          //c3_p0_wr_clk->c3_clk0
			.c3_p0_wr_en                    (c3_p0_wr_en),
         .c3_p0_wr_mask                  (c3_p0_wr_mask),
         .c3_p0_wr_data                  (c3_p0_wr_data),
         .c3_p0_wr_full                  (c3_p0_wr_full),
         .c3_p0_wr_empty                 (c3_p0_wr_empty),
         .c3_p0_wr_count                 (c3_p0_wr_count),
         .c3_p0_wr_underrun              (c3_p0_wr_underrun),
         .c3_p0_wr_error                 (c3_p0_wr_error),	
			
         // User Port-0 data read interface 
			.c3_p0_rd_clk                   (c3_clk0),          //c3_p0_rd_clk->c3_clk0
         .c3_p0_rd_en                    (c3_p0_rd_en),
         .c3_p0_rd_data                  (c3_p0_rd_data),
         .c3_p0_rd_full                  (c3_p0_rd_full),			
         .c3_p0_rd_empty                 (c3_p0_rd_empty),
         .c3_p0_rd_count                 (c3_p0_rd_count),
         .c3_p0_rd_overflow              (c3_p0_rd_overflow),
         .c3_p0_rd_error                 (c3_p0_rd_error)

       );

endmodule
