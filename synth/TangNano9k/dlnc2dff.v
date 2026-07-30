module DLNC (
    output Q,
    input D,
    input CLK,
    input CLEAR
);
    // DLNC는 Active-High Clear를 가진 Negative-Gate Latch입니다
    // (gowin/cells_sim.v: if(CLEAR) Q<=0; else if(!CLK) Q<=D;).
    // GW1N-9C(himbaechel)는 DLNC용 BEL이 부족하므로, 이를 Gowin DFFC
    // (Positive-Edge Clock, Active-High Clear D-FlipFlop)로 매핑합니다.
    // 주의: 래치의 투명(transparent) 구간 동작은 사라지고 CLK 상승 에지에서만
    // 샘플링되므로, 진짜 레벨 센서티브 동작에 의존하는 회로라면 오동작할 수 있습니다.
    DFFC _techmap_dff (
        .Q(Q),
        .D(D),
        .CLK(CLK),
        .CLEAR(CLEAR)
    );
endmodule
