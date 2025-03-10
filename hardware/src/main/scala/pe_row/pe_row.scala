package pe_row
// Reconfigurable sparse PE array에서 SDDMM과 SpMM 계산을 수행

import chisel3._
import chisel3.util._

import chisel3.experimental.FixedPoint

import exp_unit.ExpUnitFixPoint

object SeqSwitch {    // mux 역할
  def apply[T <: Data](sel: UInt, out: T, cases: IndexedSeq[Tuple2[UInt, T]]) {
    var w = when(sel === cases.head._1) {
      out := cases.head._2
    }
    for ((u, v) <- cases.tail) {
      w = w.elsewhen(sel === u) {
        out := v
      }
    }
    w.otherwise {
      out := DontCare
    }
  }
}

// Controls: (제어신호들)
// c1: input from row / score_exp
// c2: input from col / 0
// c3: stage1/stage2
// c4: bubble
// c5: exp unit

class PE(
    bits: Int,
    point: Int,
    width: Int,
    buf_size: Int,
    c1_bits: Int,
    c2_bits: Int,
    c4_bits: Int,
    id: (Int, Int)
) extends Module {
  val fpType = FixedPoint(bits.W, point.BP)
  val io = IO(new Bundle {
    val row_in = Input(Vec(width, fpType))  //[in] PE에 들어오는 행 데이터 (벡터)
    val col_in = Input(Vec(width, fpType))  //[in] PE에 들어오는 열 데이터 (벡터)
    val o_in = Input(fpType)                //[in] 이전 PE로부터 전달받은 출력
    val o_out = Output(fpType)              //[out] 현재 PE의 출력 (주로 MAC 연산 결과와 버퍼링된 값들을 기반으로 계산)
    val score_in = Input(fpType)            //[in] 이전 PE로부터 전달받은 score 값
    val score_out = Output(fpType)          //[out] 현재 PE에서 계산된 score (exp unit의 결과 등)

    val c1 = Input(UInt(c1_bits.W))
    val c2 = Input(UInt(c2_bits.W))
    val c3 = Input(UInt(2.W))
    val c4 = Input(UInt(c4_bits.W))
    val c5 = Input(UInt(2.W))
  })

  val acc_clear :: acc_idle :: acc_self :: acc_left :: Nil = Enum(4); // c3 (4가지 상태 정의)
  val exp_idle :: exp_calc :: exp_move :: Nil = Enum(3); // c5 (3가지 상태 정의)

  val exp_unit = Module(new ExpUnitFixPoint(bits, point, 6, 4))

  val a = Wire(fpType)
  val b = Wire(fpType)
  val c = Wire(fpType)
  val acc = Reg(fpType)
  val score_exp = Reg(fpType)

  // Buffer
  val buf = Reg(Vec(buf_size, fpType))
  val buf_vec = Wire(Vec(buf_size + 1, fpType))
  buf_vec(0) := acc
  for (i <- 0 until buf_size) buf_vec(i + 1) := buf(i)

  
  // MAC : row_in과 col_in에서 선택된 값을 SeqSwitch를 통해 각각 a와 b에 할당하고, 이 둘을 곱해서 c를 계산
  val a_vec = Wire(Vec(width + 2, fpType))            // width+2개의 고정소수점(fixed-point)을 담을 공간을 만든다.
  for (i <- 0 until width) a_vec(i) := io.row_in(i)   // 행 방향으로 들어오는 입력 벡터(query)를 a_vec 앞 부분에 매핑
  a_vec(width) := score_exp                           // score_exp(지수 연산 결과)를 후보 값으로 넣어둔다.
  a_vec(width + 1) := FixedPoint(0, bits.W, point.BP) // 마지막으로 0을 후보 값 중 하나로 지정
  SeqSwitch(
    io.c1,
    a,
    (Range(0, width + 2).map(_.U)).zip(a_vec)
  )
  val b_vec = Wire(Vec(width + 1, fpType))            // width+1 공간 선언
  for (i <- 0 until width) b_vec(i) := io.col_in(i)   // 열 방향으로 들어오는 입력 벡터 (key, value)를 b_vec에 매핑
  b_vec(width) := FixedPoint(0, bits.W, point.BP)     // 마지막 인덱스에 0을 넣는다.
  SeqSwitch(
    io.c2,
    b,
    Range(0, width + 1).map(_.U).zip(b_vec)
  )            // a_vec, b_vec에 여러 후보 값 담아두고, 뒤에서 SeqSwitch를 통해 a와 b에 할당될 값 선택
  c := a * b   // Multiply 연산이 일어나는 구간
  // (acc_self) : PE 내부에서 acc 레지스터에 더해지거나
  // (acc_left) : 다른 PE에서 넘어온 o_in과 합쳐지면서 누산이 진행된다.
  

  // Score
  io.score_out := score_exp

  
  // Control
  // PE 내부에서 어떤 연산을 수행할 지 결정
  // c3 : 누산기 동작 모드 전환 (4가지)
      // SDDMM(Q*K) : 같은 PE 내부에서 partial sum을 계속 누적 [iterative accumulation]
      // SpMM(S*V) : 오른쪽 PE로 결과를 넘겨가며 누적 [forward accumulation]
  io.o_out := DontCare
  switch(io.c3) {
    is(acc_clear) {          // (1) accumulation과 buffer을 0으로 초기화
      acc := FixedPoint(0, bits.W, point.BP)
      for (i <- 0 until buf.size)
        buf(i) := FixedPoint(0, bits.W, point.BP)
    }
    is(acc_idle) {           // (2) 현재 acc, buf 값을 유지 (아무 연산도 안함)
      acc := acc
      for (i <- 0 until buf.size)
        buf(i) := buf(i)
    }
    is(acc_self) {           // (3) 현재 acc에 c(=a+b)를 더한다.-> 같은 PE 내부에 partial sum을 'iterative acc'한다.
      acc := acc + c
      for (i <- 0 until buf.size)
        buf(i) := buf(i)
    }
    is(acc_left) {           // (4) io_in + c를 더해 acc 업데이트, 그리고 buffer를 시프트하여 o_out 결정
      acc := io.o_in + c     // 주로 다른 PE 출력 + 현재 곱셈 결과를 처리하는 단계 -> forward accumulation'

      buf(0) := acc
      for (i <- 0 until buf_size - 1)
        buf(i + 1) := buf(i)
      SeqSwitch(
        io.c4,
        io.o_out,
        Range(0, buf_size + 1).map(_.U).zip(buf_vec)
      )
    }
  }
  
  // c5에서 지수 연산(exp_unit, softmax 전 단계)의 동작 결정
  // SDDMM과 SpMM 연산을 하나의 모듈에서 유연하게 처리할 수 있도록 한다.
  exp_unit.io.in_value := FixedPoint(0, bits.W, point.BP)
  switch(io.c5) {
    is(exp_idle) {          // 지수 연산 수행하지 않고 score_exp register 유지
      score_exp := score_exp
    }
    is(exp_calc) {          // 누산기(acc)에 담긴 값을 exp 함수에 통과시켜, e^(acc)를 계산하여 저장하는 단계
      exp_unit.io.in_value := acc
      score_exp := exp_unit.io.out_exp
    }
    is(exp_move) {          // 이전 PE의 score_out을 현재 PE가 받아온다. (체인 형태로 PE가 연결되어 있음)
      score_exp := io.score_in
    }
  }
}

class PE_Row(
    pe_n: Int,
    bits: Int,
    point: Int,
    width: Int,
    c1_bits: Int,
    c2_bits: Int,
    c4_bits: Int,
    rowId: Int
) extends Module {
  val fpType = FixedPoint(bits.W, point.BP)
  val io = IO(new Bundle {
    val left_in = Input(fpType)                // 왼쪽에서 들어오는 단일 fixed-point 값
    val top_in = Input(Vec(width, fpType))     // 상단에서 들어오는, 길이 width인 fixed-point vector
    val bot_out = Output(Vec(width, fpType))   // 하단으로 출력되는, 길이 width인 fixed-point vector (보통 h_regs)
    val o_out = Output(fpType)                 // PE 행의 최종 출력 값 (마지막 PE의 out)
    val score_sum = Output(fpType)             // 행 전체의 score 누적 합 (모든 PE에서 전달받은 score 합계)

    val clr = Input(Bool())                    // c1, c2, c4 : 각 PE에 개별적인 선택 제어를 위해 사용
                                               // c3, c5 : 행 전체에 적용되는 제어 신호, accumulation과 exp 연산 모드 지정 
    val c1 = Input(Vec(pe_n, UInt(c1_bits.W)))
    val c2 = Input(Vec(pe_n, UInt(c2_bits.W)))
    val c3 = Input(UInt(2.W))
    val c4 = Input(Vec(pe_n, UInt(c4_bits.W)))
    val c5 = Input(UInt(2.W))
  })

  val pes =
    for (i <- 0 until pe_n)  // pe_n 개의 PE인스턴스 생성. (rowId, i) 튜플을 통해 위치 정보를 가짐
      yield Module(
        new PE(
          bits,
          point,
          width,
          width - pe_n,
          c1_bits,
          c2_bits,
          c4_bits,
          (rowId, i)
        )
      )

  val (acc_clear, acc_idle, acc_self, acc_left) = (
    pes(0).acc_clear,
    pes(0).acc_idle,
    pes(0).acc_self,
    pes(0).acc_left
  )

  val (exp_idle, exp_calc, exp_move) = (
    pes(0).exp_idle,
    pes(0).exp_calc,
    pes(0).exp_move
  );

  val v_regs = Reg(Vec(width, fpType)) // (row_in) io.left_in값이 v_regs(0)에 할당되고, 나머지 v_regs는 shift 연산으로 전달
  val h_regs = Reg(Vec(width, fpType)) // (col_in) io.top_in 값으로 갱신되어, 하단 출력 io.bot_out에도 연결된다.

  val ssum = Reg(fpType)
  io.score_sum := ssum                 // 마지막 PE의 score_out 값을 누적하여, 행 전체의 score 값을 저장하는 레지스터

  for (i <- 0 until pe_n) {
    pes(i).io.row_in := v_regs
    pes(i).io.col_in := h_regs
    pes(i).io.c1 := io.c1(i)
    pes(i).io.c2 := io.c2(i)
    pes(i).io.c3 := io.c3
    pes(i).io.c4 := io.c4(i)
    pes(i).io.c5 := io.c5
  }
  for (i <- 0 until pe_n - 1) {         // score_in과 o_in이 이전 PE(i)의 score_out과 o_out에 연결된다.
    pes(i + 1).io.score_in := pes(i).io.score_out
    pes(i + 1).io.o_in := pes(i).io.o_out
  } // 첫 번째 PE는 초기값으로 0 할당
  pes(0).io.score_in := FixedPoint(0, bits.W, point.BP)
  pes(0).io.o_in := FixedPoint(0, bits.W, point.BP)

  for (i <- 0 until width)
    io.bot_out(i) := h_regs(i)

  io.o_out := pes(pe_n - 1).io.o_out  // 최종 출력 io.o_out은 마지막 PE(pe_n-1)의 o_out 값이다.

  when(io.clr) {
    for (i <- 0 until width) {
      v_regs(i) := FixedPoint(0, bits.W, point.BP)
      h_regs(i) := FixedPoint(0, bits.W, point.BP)
    }
    ssum := FixedPoint(0, bits.W, point.BP)
  }.otherwise {
    for (i <- 0 until width - 1)
      v_regs(i + 1) := v_regs(i)  // v_regs 업데이트 (shift)
    v_regs(0) := io.left_in       // (0)은 외부 입력으로 갱신

    h_regs := io.top_in           // 상단 입력 벡터가 h_regs에 복사
    io.bot_out := h_regs          // 행의 하단 출력 구성

    when(io.c5 === exp_move) {    // score 누적 (ssum 업데이트)
      ssum := ssum + pes(pe_n - 1).io.score_out  // io.c5가 exp_move 상태이면, 마지막 PE의 score_out 값을 ssum에 누적
    }.otherwise {
      ssum := ssum
    }
  }
}

class Sparse_PE_Array(    // 행 단위로 여러 row를 쌓아서 전체 systolic array를 형성
                          // attention 연산을 수행할 때 각 행 별로 계산된 결과를 종합하고, 상위 시스템에 최종 결과와 score 합계를 제공
    pe_n: Int,            // 각 PE_Row에 포함된 PE의 개수 (한 행의 PE 수)
    bits: Int,            // bits, point : fixed point 표현을 위한 비트 폭 및 소수점 위치
    point: Int,
    width: Int,           // 각 PE_row의 데이터 폭 (PE가 처리할 벡터의 길이)
    height: Int,          // 전체 PE 배열의 행 수
    c1_bits: Int,         // c1, c2, c4 : 제어 신호의 비트 폭 (각 PE에 개별적으로 전달될 제어 신호들)
    c2_bits: Int,
    c4_bits: Int
) extends Module {
  val fpType = FixedPoint(bits.W, point.BP)
  val io = IO(new Bundle {
    val left_in = Input(Vec(height, fpType))    // 길이 height인 벡터로, 각 행(PE_row)별 왼쪽에서 들어오는 값
    val top_in = Input(Vec(width, fpType))      // 길이 width인 벡터로, 최상단 행의 상단 입력값
    val out = Output(Vec(height, fpType))         // 길이 height인 벡터, 각 행(PE_row)의 최종 출력값
    val score_sum = Output(Vec(height, fpType))   // 길이 height인 벡터, 각 행에서 누적된 score의 합계를 출력
    val clr = Input(Bool())
    val c1 = Input(Vec(height, Vec(pe_n, UInt(c1_bits.W))))  // 각 행의 PE별로 전달될 제어 신호들
    val c2 = Input(Vec(height, Vec(pe_n, UInt(c2_bits.W))))
    val c3 = Input(UInt(2.W))      // 2비트 제어 신호로, 모든 행에 동일하게 적용되는 누산 모드 
    val c4 = Input(Vec(height, Vec(pe_n, UInt(c4_bits.W))))
    val c5 = Input(UInt(2.W))      // 2비트 제어 신호로, 모든 행에 동일하게 적용되는 exp 연산 모드
  })
  val rows =                    
    for (i <- 0 until height)     //row라는 벡터를 생성하여, height 개의 PE_row 모듈 인스턴스를 만든다.
      yield Module(
        new PE_Row(
          pe_n,
          bits,             
          point,
          width,            
          c1_bits,         
          c2_bits,
          c4_bits,
          i
        )
      )
  val (acc_clear, acc_idle, acc_self, acc_left) = (
    rows(0).acc_clear,
    rows(0).acc_idle,
    rows(0).acc_self,
    rows(0).acc_left
  )

  val (exp_idle, exp_calc, exp_move) = (
    rows(0).exp_idle,
    rows(0).exp_calc,
    rows(0).exp_move
  );

  for (i <- 0 until height) {  // 제어 신호 벡터 (c1, c2, c4) , 공통 제어 신호 (c3, c5)
    rows(i).io.clr := io.clr
    rows(i).io.c1 := io.c1(i)
    rows(i).io.c2 := io.c2(i)
    rows(i).io.c3 := io.c3
    rows(i).io.c4 := io.c4(i)
    rows(i).io.c5 := io.c5
    rows(i).io.left_in := io.left_in(i)
    io.out(i) := rows(i).io.o_out
    io.score_sum(i) := rows(i).io.score_sum
  }
  rows(0).io.top_in := io.top_in
  for (i <- 0 until height - 1)                 // 이전 행의 하단 출력(bot_out)이 다음 행의 상단 입력(top_in)으로 전달
    rows(i + 1).io.top_in := rows(i).io.bot_out // PE_Row 간의 데이터 흐름이 수직으로 이어지게 된다.
}                                       // 이를 통해, 전체 PE 배열은 한 행의 결과가 다음 행으로 전달되는 형태의 systolic dataflow 형성
