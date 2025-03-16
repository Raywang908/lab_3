module fir 
#(  parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 12
)
(
    output  wire                     awready,
    output  wire                     wready,
    input   wire                     awvalid,
    input   wire [(pADDR_WIDTH-1):0] awaddr,
    input   wire                     wvalid,
    input   wire [(pDATA_WIDTH-1):0] wdata,
    output  wire                     arready,
    input   wire                     rready,
    input   wire                     arvalid,
    input   wire [(pADDR_WIDTH-1):0] araddr,
    output  wire                     rvalid,
    output  wire [(pDATA_WIDTH-1):0] rdata,    
    input   wire                     ss_tvalid, 
    input   wire [(pDATA_WIDTH-1):0] ss_tdata, 
    input   wire                     ss_tlast, 
    output  wire                     ss_tready, 
    input   wire                     sm_tready, 
    output  wire                     sm_tvalid, 
    output  wire [(pDATA_WIDTH-1):0] sm_tdata, 
    output  wire                     sm_tlast, 
    
    // bram for tap RAM
    output  wire [3:0]               tap_WE,
    output  wire                     tap_EN,
    output  wire [(pDATA_WIDTH-1):0] tap_Di,
    output  wire [(pADDR_WIDTH-1):0] tap_A,
    input   wire [(pDATA_WIDTH-1):0] tap_Do,

    // bram for data RAM
    output  wire [3:0]               data_WE,
    output  wire                     data_EN,
    output  wire [(pDATA_WIDTH-1):0] data_Di,
    output  wire [(pADDR_WIDTH-1):0] data_A,
    input   wire [(pDATA_WIDTH-1):0] data_Do,

    input   wire                     axis_clk,
    input   wire                     axis_rst_n
);

//========================== Declaration ==========================
//-------------------------- Axi-Lite -------------------------------
  reg rvalid_next;
  localparam WRITE = 2'b00;
  localparam READ = 2'b01;
  localparam IDLE = 2'b10;
  localparam NULL_ADDR = 12'h90;
  reg [1:0] axi_state; 
  reg [1:0] axi_state_next; //1.first read -> wait write 2. read/write same time -> if not wait/waiting -> write/read 3.
  reg axi_state_finish; 
  reg awready_tmp;
  reg wready_tmp;
  reg arready_tmp;
  reg rvalid_tmp;
  reg [(pADDR_WIDTH-1):0] awaddr_tmp;
  reg [(pADDR_WIDTH-1):0] araddr_tmp;
  reg [(pADDR_WIDTH-1):0] araddr_next;
  reg [(pDATA_WIDTH-1):0] rdata_tmp;
  reg tap_EN_tmp;
  reg [3:0] tap_WE_tmp;
  reg [(pADDR_WIDTH-1):0] tap_A_tmp;
  reg [(pDATA_WIDTH-1):0] tap_Di_tmp;
  localparam ADDR_MASK = 32'd127;
  reg write_alr;
  reg write_tap_next;
  reg write_tap;
  reg read_tap;
  reg read_tap_next;
  reg [(pDATA_WIDTH-1):0] data_length;
  reg [(pDATA_WIDTH-1):0] data_length_next;
  reg [5:0] tap_length; //[(pDATA_WIDTH-1):0]
  reg [5:0] tap_length_next;
  wire [(pDATA_WIDTH-1):0] tap_length_large;
  reg axi_lite_on;
  reg tap_EN_all;
  reg [3:0] tap_WE_all;
  reg [(pADDR_WIDTH-1):0] tap_A_all;
  reg [(pDATA_WIDTH-1):0] tap_Di_all;
//-------------------------- ap_idle & ap_done & ap_start -------------------------------
  reg [2:0] ap_ctrl; //[2] -> ap_idle,[1] -> ap_done,[0] -> ap_start
  reg ap_idle_next;
  reg ap_done_next;
  reg done_read_next;
  reg done_read;
  reg ap_start_next;
//-------------------------- Axi-Stream SS (input X) -------------------------------
  reg [1:0] ss_state;
  reg [1:0] ss_state_next;
  localparam IDLE_SS = 2'b10;
  localparam READ_SS = 2'b01;
  localparam WRITE_SS = 2'b00;
  reg [(pADDR_WIDTH-1):0] data_A_tmp;
  reg [(pDATA_WIDTH-1):0] data_Di_tmp;
  reg data_EN_tmp;
  reg [3:0] data_WE_tmp;
  reg ss_tready_tmp;
  reg ss_tready_next;
  reg [(pDATA_WIDTH-1):0] data_cnt;
  reg [(pDATA_WIDTH-1):0] data_cnt_next;
  reg read_should;
  reg read_should_next;
  reg ss_on;
  reg tap_EN_ss;
  reg [3:0] tap_WE_ss;
  reg [(pADDR_WIDTH-1):0] tap_A_ss;
  reg [(pDATA_WIDTH-1):0] tap_Di_ss;
  reg [5:0] operation;
  reg [5:0] current_next;
  reg [5:0] current;
  reg wait_sm;
  reg wait_sm_next;
  reg [1:0] toread_y_cnt;
  reg [1:0] toread_y_cnt_next;
  reg [(pDATA_WIDTH-1):0] stop_early_next;
  reg [(pDATA_WIDTH-1):0] stop_early;
  reg lock_off;
  reg lock_off_next;
//-------------------------- Axi-Stream SM (input Y) -------------------------------
  reg sm_tvalid_tmp;
  reg sm_tvalid_next;
  reg send_should_next;
  reg send_should;
  reg [(pDATA_WIDTH-1):0] sm_tdata_tmp;
  reg [(pDATA_WIDTH-1):0] sm_tdata_next;
  reg send_waiting;
  reg send_waiting_next;
//-------------------------- FIR convolution -------------------------------
  reg [(pDATA_WIDTH-1):0] data_in_conv;
  reg [(pDATA_WIDTH-1):0] tap_in_conv;
  reg [(pDATA_WIDTH-1):0] data_in_conv_next;
  reg [(pDATA_WIDTH-1):0] tap_in_conv_next;
  reg [(pDATA_WIDTH-1):0] h;
  reg [(pDATA_WIDTH-1):0] x;
  reg [(pDATA_WIDTH-1):0] y;
  wire [(pDATA_WIDTH-1):0] y_tmp;
  reg [(pDATA_WIDTH-1):0] y_next;
  reg [(pDATA_WIDTH-1):0] m;
  wire [(pDATA_WIDTH-1):0] m_tmp;
  reg [(pDATA_WIDTH-1):0] output_final;
//-------------------------- Addr generator -------------------------------
  reg [(pADDR_WIDTH-1):0] addr_genr_next;
  reg [(pADDR_WIDTH-1):0] addr_genr;
  reg [(pADDR_WIDTH-1):0] tap_genr_next;
  reg [(pADDR_WIDTH-1):0] tap_genr;

//========================== Function ==========================
//-------------------------- Axi-Lite -------------------------------

  always @(*) begin
    if (arvalid && ((araddr == 12'h10) || (araddr == 12'h14) || (araddr == 12'd0))) begin //QS: arvalid can be take out
      araddr_next = araddr;
      read_tap_next = 0;
    end else if (arvalid && araddr[7]) begin
      araddr_next = araddr & ADDR_MASK;
      read_tap_next = 1;
    end else if (arvalid) begin
      araddr_next = NULL_ADDR;
      read_tap_next = 1;
    end else begin
      araddr_next = araddr_tmp;
      read_tap_next = read_tap; 
    end
  end

  always @(*) begin
    if ((awaddr == 12'h10) || (awaddr == 12'h14) || ((awaddr == 12'd0) && ap_ctrl[2] && (axi_state_next == WRITE))) begin
      awaddr_tmp = awaddr;
      write_tap_next = 0;
    end else if (awaddr[7] == 1) begin
      awaddr_tmp = awaddr & ADDR_MASK;
      write_tap_next = 1;
    end else begin
      awaddr_tmp = NULL_ADDR;
      write_tap_next = 1; //QS
    end
  end 

  always @(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n) begin
      axi_state <= IDLE;
      araddr_tmp <= NULL_ADDR;
      rvalid_tmp <= 0;
      write_tap <= 1;
      read_tap <= 1;
      done_read <= 0;
      data_length <= 0;
      tap_length <= 0;
    end else begin
      axi_state <= axi_state_next;
      araddr_tmp <= araddr_next;
      rvalid_tmp <= rvalid_next;
      write_tap <= write_tap_next;
      read_tap <= read_tap_next;
      done_read <= done_read_next;
      data_length <= data_length_next;
      tap_length <= tap_length_next;
    end
  end

  assign rvalid = rvalid_tmp;
  assign arready = arready_tmp;


  always @(*) begin
    /*
    if (arvalid && awvalid && wvalid && axi_state_finish) begin
      if (write_alr) begin
        axi_state_next = READ;
      end else begin
        axi_state_next = WRITE;
      end
    end else 
    */
    if (awvalid && wvalid && axi_state_finish) begin
      if (write_alr) begin
        axi_state_next = IDLE;
      end else begin
        axi_state_next = WRITE;
      end
    end else if (arvalid && axi_state_finish) begin
      axi_state_next = READ;
    end else if (!axi_state_finish) begin
      axi_state_next = axi_state;
    end else begin
      axi_state_next = IDLE;
    end
  end

  always @(*) begin
    case (axi_state)
    WRITE: begin
      axi_lite_on = 1;
      rdata_tmp = 32'd0;
      rvalid_next = 0;
      arready_tmp = 0;
      done_read_next = 0; //done_read_next = done_read;
      if (!ap_ctrl[2] && write_tap) begin  
        wready_tmp = 1;
        awready_tmp = 1;
        tap_EN_tmp = 0;
        tap_WE_tmp = 4'b1111;
        tap_A_tmp = awaddr_tmp;
        tap_Di_tmp = wdata;
        axi_state_finish = 1;
        write_alr = 1;
        data_length_next = data_length;
        tap_length_next = tap_length;
        ap_start_next = 0; //ap_ctrl[0]
      end else if (write_tap == 1) begin
        wready_tmp = 1;
        awready_tmp = 1;
        tap_EN_tmp = 1;
        tap_WE_tmp = 4'b1111;
        tap_A_tmp = awaddr_tmp;
        tap_Di_tmp = wdata;
        axi_state_finish = 1;
        write_alr = 1;
        data_length_next = data_length;
        tap_length_next = tap_length;
        ap_start_next = 0; //ap_ctrl[0]
      end else begin
        tap_EN_tmp = 0;
        tap_WE_tmp = 0;
        tap_A_tmp = NULL_ADDR;
        tap_Di_tmp = 0;
        if (awaddr_tmp == 12'h10) begin
          wready_tmp = 1;
          awready_tmp = 1;
          if (!ap_ctrl[2]) begin
            data_length_next = data_length;
          end else begin
            data_length_next = wdata; 
          end
          tap_length_next = tap_length;
          axi_state_finish = 1;
          write_alr = 1;
          ap_start_next = 0; //ap_ctrl[0]
        end else if (awaddr_tmp == 12'h14) begin
          wready_tmp = 1;
          awready_tmp = 1;
          data_length_next = data_length;
          if (!ap_ctrl[2]) begin
            tap_length_next = tap_length;
          end else begin
            tap_length_next = wdata; 
          end
          axi_state_finish = 1;
          write_alr = 1;
          ap_start_next = 0; //ap_ctrl[0]
        end else begin
          wready_tmp = 1;
          awready_tmp = 1;
          data_length_next = data_length;
          tap_length_next = tap_length;
          //ap_start_next = 1; 
          if (!ap_ctrl[2]) begin
            ap_start_next = 0;
          end else begin
            ap_start_next = 1;
          end
          axi_state_finish = 1;
          write_alr = 1;
        end
      end
    end

    READ: begin
      axi_lite_on = 1;
      write_alr = 0;
      wready_tmp = 0;
      awready_tmp = 0;
      //arready_tmp = 1;
      data_length_next = data_length;
      tap_length_next = tap_length;
      ap_start_next = 0; //ap_ctrl[0]
      if (!ap_ctrl[2] && read_tap) begin //(!ap_ctrl[2] || (araddr_tmp == NULL_ADDR))
        done_read_next = 0; //done_read_next = done_read;
        if (arvalid) begin //arready && arvalid
          arready_tmp = 1;
          rdata_tmp = 32'd0;
          rvalid_next = 1;
          tap_EN_tmp = 0;
          tap_WE_tmp = 0;
          tap_A_tmp = araddr_tmp;
          tap_Di_tmp = wdata;
          axi_state_finish = 0;
        end else if (rready && rvalid) begin
          arready_tmp = 0;
          rdata_tmp = 32'hffffffff;
          rvalid_next = 0;
          tap_EN_tmp = 0;
          tap_WE_tmp = 0;
          tap_A_tmp = araddr_tmp;
          tap_Di_tmp = wdata;
          axi_state_finish = 1;
        end else begin
          arready_tmp = 0;
          rdata_tmp = 32'd0;
          rvalid_next = rvalid_tmp;
          tap_EN_tmp = 0;
          tap_WE_tmp = 0;
          tap_A_tmp = araddr_tmp;
          tap_Di_tmp = wdata;
          axi_state_finish = 0;
        end
      end else if (!read_tap) begin
        tap_EN_tmp = 0;
        tap_WE_tmp = 0;
        tap_A_tmp = NULL_ADDR;
        tap_Di_tmp = 0;
        if (arvalid) begin //arready && arvalid
          arready_tmp = 1;
          rdata_tmp = 32'd0;
          rvalid_next = 1;
          axi_state_finish = 0;
          done_read_next = 0;
        end else if (rready && rvalid) begin
          arready_tmp = 0;
          rvalid_next = 0;
          if (araddr_tmp == 12'h10) begin
            rdata_tmp = data_length;
          end else if (araddr_tmp == 12'h14) begin
            rdata_tmp = tap_length;
          end else begin
            rdata_tmp = ap_ctrl;
          end
          axi_state_finish = 1;
          if (ap_ctrl[2] && ap_ctrl[1]) begin
            done_read_next = 1;
          end else begin
            done_read_next = 0;
          end
        end else begin
          arready_tmp = 0;
          rdata_tmp = 32'd0;
          rvalid_next = rvalid_tmp;
          axi_state_finish = 0;
          done_read_next = 0; //done_read_next = done_read;
        end
      end else begin
        done_read_next = 0; //done_read_next = done_read;
        if (arvalid) begin //arready && arvalid
          arready_tmp = 1;
          rdata_tmp = 32'd0;
          rvalid_next = 1;
          tap_EN_tmp = 0;
          tap_WE_tmp = 0;
          tap_A_tmp = araddr_tmp;
          tap_Di_tmp = wdata;
          axi_state_finish = 0;
        end else if (rready && rvalid) begin
          arready_tmp = 0;
          rvalid_next = 0;
          tap_EN_tmp = 1;
          tap_WE_tmp = 0;
          tap_A_tmp = araddr_tmp;
          tap_Di_tmp = wdata;
          rdata_tmp = tap_Do;
          axi_state_finish = 1;
        end else begin
          arready_tmp = 0;
          rdata_tmp = 32'd0;
          rvalid_next = rvalid_tmp;
          tap_EN_tmp = 0;
          tap_WE_tmp = 0;
          tap_A_tmp = araddr_tmp;
          tap_Di_tmp = wdata;
          axi_state_finish = 0;
        end
      end
    end

    IDLE: begin
      axi_lite_on = 0;
      write_alr = 0;
      rvalid_next = 0;
      rdata_tmp = 32'd0;
      tap_EN_tmp = 0;
      tap_WE_tmp = 0;
      tap_A_tmp = NULL_ADDR;
      tap_Di_tmp = 0;
      arready_tmp = 0;
      wready_tmp = 0;
      awready_tmp = 0;
      axi_state_finish = 1;
      data_length_next = data_length;
      tap_length_next = tap_length;
      ap_start_next = 0; //ap_ctrl[0]
      done_read_next = 0; //done_read_next = done_read;
    end

    default: begin
      axi_lite_on = 0;
      write_alr = 0;
      rvalid_next = 0;
      rdata_tmp = 32'd0;
      tap_EN_tmp = 0;
      tap_WE_tmp = 0;
      tap_A_tmp = NULL_ADDR;
      tap_Di_tmp = 0;
      arready_tmp = 0;
      wready_tmp = 0;
      awready_tmp = 0;
      axi_state_finish = 1;
      data_length_next = data_length;
      tap_length_next = tap_length;
      ap_start_next = 0; //ap_ctrl[0]
      done_read_next = 0; //done_read_next = done_read;
    end
    endcase
  end

  assign rdata = rdata_tmp;
  assign awready = awready_tmp;
  assign wready = wready_tmp;

  always @(*) begin
    if (axi_lite_on && !ss_on) begin
      tap_A_all = tap_A_tmp;
      tap_Di_all = tap_Di_tmp;
      tap_EN_all = tap_EN_tmp;
      tap_WE_all = tap_WE_tmp;
    end else begin
      tap_A_all = tap_A_ss;
      tap_Di_all = tap_Di_ss;
      tap_EN_all = tap_EN_ss;
      tap_WE_all = tap_WE_ss;      
    end
  end

  assign tap_A = tap_A_all;
  assign tap_Di = tap_Di_all;
  assign tap_EN = tap_EN_all;
  assign tap_WE = tap_WE_all;



//-------------------------- ap_idle & ap_done & ap_start -------------------------------

  always @(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n) begin
      ap_ctrl[2] <= 1;
      ap_ctrl[1] <= 0;
      ap_ctrl[0] <= 0;
    end else begin
      ap_ctrl[2] <= ap_idle_next;
      ap_ctrl[1] <= ap_done_next;
      ap_ctrl[0] <= ap_start_next;
    end
  end

  always @(*) begin
    if (ap_ctrl[0] == 1) begin
      ap_idle_next = 0;
    end else if (sm_tlast == 1) begin
      ap_idle_next = 1;
    end else begin
      ap_idle_next = ap_ctrl[2];
    end

    if (sm_tlast == 1 && !ap_ctrl[2]) begin
      ap_done_next = 1;
    end else if (ap_ctrl[2] && done_read) begin
      ap_done_next = 0;
    end else begin
      ap_done_next = ap_ctrl[1];
    end
  end

//-------------------------- Axi-Stream SS (input X) -------------------------------

  always @(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n) begin
      data_cnt <= 0;
      ss_state <= IDLE_SS;
      ss_tready_tmp <= 0;
      read_should <= 0;
      current <= 0;
      wait_sm <= 0;
      stop_early <= (1 << pDATA_WIDTH) - 1;
      lock_off <= 1;
    end else begin
      data_cnt <= data_cnt_next;
      ss_state <= ss_state_next;
      ss_tready_tmp <= ss_tready_next;
      read_should <= read_should_next;
      current <= current_next;
      wait_sm <= wait_sm_next;
      stop_early <= stop_early_next;
      lock_off <= lock_off_next;
    end
  end

  assign tap_length_large = tap_length;

  always @(*) begin
    if (data_cnt < tap_length_large) begin
      operation = data_cnt;
    end else begin
      operation = tap_length - 1;
    end
  end

  always @(*) begin
    if (ss_tlast && lock_off) begin
      stop_early_next = data_cnt + 1;
      lock_off_next = 0; 
    end else if (ap_ctrl[2] && done_read == 1) begin
      stop_early_next = (1 << pDATA_WIDTH) - 1; //QS:
      lock_off_next = 1;
    end else begin
      stop_early_next = stop_early;
      lock_off_next = lock_off;
    end
  end

  //reg [4:0] debug_ss;
  always @(*) begin
    if ((data_cnt <= stop_early) && !ap_ctrl[2]) begin  // wait_sm to let y_tmp get //&& !ss_tlast // (data_cnt <= data_length) &&
      ss_on = 1;
      //debug_ss = 1;
      if (ss_tvalid && data_cnt == 0 && ss_tdata != 0) begin
        ss_state_next = WRITE_SS;
        data_cnt_next = data_cnt + 1;
        ss_tready_next = 1;
        current_next = 0;
        addr_genr_next = 0;
        tap_genr_next = 0; //(tap_length - 1) << 2
        wait_sm_next = 1;
        //debug_ss = 0;
      end else if (ss_tvalid && (current == operation) && !send_waiting_next) begin //!send_waiting && ss_tdata != 0
        ss_state_next = WRITE_SS;
        data_cnt_next = data_cnt + 1;
        ss_tready_next = 1;
        current_next = 0;
        wait_sm_next = 1;
        //debug_ss = 1;
        if (addr_genr == ((tap_length - 1) << 2)) begin
          addr_genr_next = 0;
        end else begin
          addr_genr_next = addr_genr + 3'd4;
        end
        if (tap_genr == 0) begin
          tap_genr_next = ((tap_length - 1) << 2);
        end else begin
          tap_genr_next = tap_genr - 3'd4;
        end
      end else if ((current < operation) && current == 0 && !wait_sm && !send_waiting_next) begin
        ss_state_next = READ_SS;
        data_cnt_next = data_cnt;
        ss_tready_next = 0;
        current_next = current + 1;
        wait_sm_next = 0;
        //debug_ss = 2;
        if (data_cnt < tap_length_large) begin
          addr_genr_next = current;
          tap_genr_next = (operation << 2); //operation * 3'd4
        end else begin
          addr_genr_next = ((data_cnt - tap_length + 1) % tap_length) << 2; //((data_cnt - tap_length + 1) % tap_length) * 3'd4
          tap_genr_next = ((tap_length - 1) << 2); //0
        end
      end else if (current < operation && !wait_sm && !send_waiting_next) begin
        ss_state_next = READ_SS;
        data_cnt_next = data_cnt;
        ss_tready_next = 0;
        current_next = current + 1;
        wait_sm_next = 0;
        //debug_ss = 3;
        if (addr_genr == ((tap_length - 1) << 2)) begin
          addr_genr_next = 0;
        end else begin
          addr_genr_next = addr_genr + 3'd4;
        end
        if (tap_genr == 0) begin
          tap_genr_next = ((tap_length - 1) << 2);
        end else begin
          tap_genr_next = tap_genr - 3'd4;
        end
      end else begin
        ss_state_next = IDLE_SS;
        data_cnt_next = data_cnt;
        ss_tready_next = 0;
        current_next = current;
        addr_genr_next = addr_genr;
        tap_genr_next = tap_genr;
        wait_sm_next = 0;
        //debug_ss = 4;
      end
    end else begin
      ss_state_next = IDLE_SS;
      if (done_read == 1) begin
        data_cnt_next = 0; //QS:
      end else begin
        data_cnt_next = data_cnt;
      end
      ss_tready_next = 0;
      current_next = 0;
      ss_on = 0;
      addr_genr_next = NULL_ADDR;
      tap_genr_next = NULL_ADDR;
      wait_sm_next = 0;
      //debug_ss = 5;
    end
  end

  assign ss_tready = ss_tready_tmp;

  always @(*) begin
    case (ss_state)
      WRITE_SS: begin
        data_A_tmp = addr_genr;
        data_Di_tmp = ss_tdata;
        data_EN_tmp = 1;
        data_WE_tmp = 4'b1111;
        data_in_conv = data_Do;

        tap_A_ss = tap_genr;
        tap_Di_ss = 0;
        tap_WE_ss = 0;
        tap_EN_ss = 0;
        tap_in_conv = tap_Do;
        read_should_next = 1;
        if (read_should) begin
          tap_EN_ss = 1;
        end else begin
          tap_EN_ss = 0;
        end
      end

      READ: begin
        data_A_tmp = addr_genr;
        data_Di_tmp = 0;
        data_WE_tmp = 0;
        data_in_conv = data_Do;

        tap_A_ss = tap_genr;
        tap_Di_ss = 0;
        tap_WE_ss = 0;
        tap_in_conv = tap_Do;
        if (read_should) begin
          tap_EN_ss = 1;
          data_EN_tmp = 1;
        end else begin
          tap_EN_ss = 0;
          data_EN_tmp = 0;
        end
        read_should_next = 1;
      end

      IDLE_SS: begin
        data_A_tmp = NULL_ADDR;
        data_Di_tmp = 0;
        data_WE_tmp = 0;
        data_in_conv = data_Do;

        tap_A_ss = NULL_ADDR;
        tap_Di_ss = 0;
        tap_WE_ss = 0;
        tap_in_conv = tap_Do;
        if (read_should) begin
          tap_EN_ss = 1;
          data_EN_tmp = 1;
        end else begin
          tap_EN_ss = 0;
          data_EN_tmp = 0;
        end
        read_should_next = 0;
      end

      default: begin
        data_A_tmp = NULL_ADDR;
        data_Di_tmp = 0;
        data_EN_tmp = 0;
        data_WE_tmp = 0;
        data_in_conv = 0;

        tap_A_ss = NULL_ADDR;
        tap_Di_ss = 0;
        tap_WE_ss = 0;
        tap_EN_ss = 0;
        tap_in_conv = 0;
        read_should_next = 0;
      end
    endcase
  end

  assign data_A = data_A_tmp;
  assign data_Di = data_Di_tmp;
  assign data_EN = data_EN_tmp;
  assign data_WE = data_WE_tmp;
//-------------------------- Axi-Stream SM (input Y) -------------------------------
  
  assign sm_tdata = sm_tdata_tmp;
  assign sm_tvalid = sm_tvalid_tmp;
  assign sm_tlast = ((data_cnt >= stop_early) && sm_tvalid) ? 1 : 0; //&& send_should

  always @(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n) begin
      sm_tvalid_tmp <= 0;
      sm_tdata_tmp <= 0;
      send_should <= 0;
      send_waiting <= 0;
      toread_y_cnt <= 0;
    end else begin
      sm_tvalid_tmp <= sm_tvalid_next;
      sm_tdata_tmp <= sm_tdata_next;
      send_should <= send_should_next;
      send_waiting <= send_waiting_next;
      toread_y_cnt <= toread_y_cnt_next;
    end
  end

  always @(*) begin
    if (!ap_ctrl[2] && (data_cnt <= stop_early)) begin //sm_tlast
      if (sm_tready && send_should || sm_tready && send_waiting) begin // 
        sm_tvalid_next = 1;
        sm_tdata_next = output_final;
        send_waiting_next = 0;
      end else if (send_should) begin //&& !(data_count <= tap_length)
        sm_tvalid_next = 0;
        sm_tdata_next = output_final;
        send_waiting_next = 1;
      end else begin
        sm_tvalid_next = 0;
        sm_tdata_next = 0;
        send_waiting_next = send_waiting;
      end
    end else begin
      sm_tvalid_next = 0;
      sm_tdata_next = 0;
      send_waiting_next = 0;
    end
  end

//-------------------------- FIR convolution -------------------------------

  assign m_tmp = (read_should) ? x * h : 0;
  assign y_tmp = y + m;

  always @(*) begin
    if (ss_state == WRITE_SS) begin
      toread_y_cnt_next = 1;
    end else if (toread_y_cnt == 1) begin
      toread_y_cnt_next = 2;
    end else begin
      toread_y_cnt_next = 0;
    end
  end

  always @(*) begin
    if (toread_y_cnt == 2) begin //&& !(y_tmp == 0)
      y_next = y_tmp;
      send_should_next = 1;
    end else begin
      y_next = output_final;
      send_should_next = 0;
    end
  end

  always @(*) begin
    if (read_should) begin
      x = data_in_conv;
      h = tap_in_conv;
    end else begin
      x = 0;
      h = 0;
    end
  end

  always @(posedge axis_clk or negedge axis_rst_n) begin
    if (!axis_rst_n) begin
      m <= 0;
      y <= 0;
      output_final <= 0;
    end else begin
      m <= m_tmp;
      output_final <= y_next;
      if (send_should_next == 1 || ap_ctrl[2]) begin //QS:
        y <= 0;
      end else begin
        y <= y_tmp;
      end
    end
  end

//-------------------------- Addr generator -------------------------------

always @(posedge axis_clk or negedge axis_rst_n) begin
  if (!axis_rst_n) begin
    addr_genr <= NULL_ADDR;
    tap_genr <= NULL_ADDR;
  end else begin
    addr_genr <= addr_genr_next;
    tap_genr <= tap_genr_next;
  end
end


endmodule