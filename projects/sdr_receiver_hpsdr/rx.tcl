# Create axi_hub
cell pavel-demin:user:axi_hub hub_0 {
  CFG_DATA_WIDTH 288
  STS_DATA_WIDTH 32
} {
  aclk /pll_0/clk_out1
  aresetn /rst_0/peripheral_aresetn
}

# Create port_slicer
cell pavel-demin:user:port_slicer slice_0 {
  DIN_WIDTH 288 DIN_FROM 0 DIN_TO 0
} {
  din hub_0/cfg_data
}

# Create port_slicer
cell pavel-demin:user:port_slicer slice_1 {
  DIN_WIDTH 288 DIN_FROM 31 DIN_TO 16
} {
  din hub_0/cfg_data
}

for {set i 0} {$i <= 7} {incr i} {

  # Create port_slicer
  cell pavel-demin:user:port_slicer slice_[expr $i + 2] {
    DIN_WIDTH 288 DIN_FROM [expr $i + 8] DIN_TO [expr $i + 8]
  } {
    din hub_0/cfg_data
  }

  # Create port_selector
  cell pavel-demin:user:port_selector selector_$i {
    DOUT_WIDTH 16
  } {
    cfg slice_[expr $i + 2]/dout
    din /adc_0/m_axis_tdata
  }

  # Create port_slicer
  cell pavel-demin:user:port_slicer slice_[expr $i + 10] {
    DIN_WIDTH 288 DIN_FROM [expr 32 * $i + 63] DIN_TO [expr 32 * $i + 32]
  } {
    din hub_0/cfg_data
  }

  # Create axis_constant
  cell pavel-demin:user:axis_constant phase_$i {
    AXIS_TDATA_WIDTH 32
  } {
    cfg_data slice_[expr $i + 10]/dout
    aclk /pll_0/clk_out1
  }

  # Create dds_compiler
  cell xilinx.com:ip:dds_compiler dds_$i {
    DDS_CLOCK_RATE 125
    SPURIOUS_FREE_DYNAMIC_RANGE 120
    FREQUENCY_RESOLUTION 0.2
    PHASE_INCREMENT Streaming
    HAS_PHASE_OUT false
    PHASE_WIDTH 30
    OUTPUT_WIDTH 21
    DSP48_USE Minimal
    NEGATIVE_SINE true
  } {
    S_AXIS_PHASE phase_$i/M_AXIS
    aclk /pll_0/clk_out1
  }

}

# Create xlconstant
cell xilinx.com:ip:xlconstant const_0

for {set i 0} {$i <= 15} {incr i} {

  # Create port_slicer
  cell pavel-demin:user:port_slicer dds_slice_$i {
    DIN_WIDTH 48 DIN_FROM [expr 24 * ($i % 2) + 20] DIN_TO [expr 24 * ($i % 2)]
  } {
    din dds_[expr $i / 2]/m_axis_data_tdata
  }

  # Create dsp48
  cell pavel-demin:user:dsp48 mult_$i {
    A_WIDTH 21
    B_WIDTH 14
    P_WIDTH 24
  } {
    A dds_slice_$i/dout
    B selector_[expr $i / 2]/dout
    CLK /pll_0/clk_out1
  }

  # Create axis_variable
  cell pavel-demin:user:axis_variable rate_$i {
    AXIS_TDATA_WIDTH 16
  } {
    cfg_data slice_1/dout
    aclk /pll_0/clk_out1
    aresetn /rst_0/peripheral_aresetn
  }

  # Create cic_compiler
  cell xilinx.com:ip:cic_compiler cic_$i {
    INPUT_DATA_WIDTH.VALUE_SRC USER
    FILTER_TYPE Decimation
    NUMBER_OF_STAGES 6
    SAMPLE_RATE_CHANGES Programmable
    MINIMUM_RATE 250
    MAXIMUM_RATE 2000
    FIXED_OR_INITIAL_RATE 500
    INPUT_SAMPLE_FREQUENCY 125
    CLOCK_FREQUENCY 125
    INPUT_DATA_WIDTH 24
    QUANTIZATION Truncation
    OUTPUT_DATA_WIDTH 32
    USE_XTREME_DSP_SLICE false
    HAS_ARESETN true
  } {
    s_axis_data_tdata mult_$i/P
    s_axis_data_tvalid const_0/dout
    S_AXIS_CONFIG rate_$i/M_AXIS
    aclk /pll_0/clk_out1
    aresetn /rst_0/peripheral_aresetn
  }

}

# Create axis_combiner
cell  xilinx.com:ip:axis_combiner comb_0 {
  TDATA_NUM_BYTES.VALUE_SRC USER
  TDATA_NUM_BYTES 4
  NUM_SI 16
} {
  S00_AXIS cic_0/M_AXIS_DATA
  S01_AXIS cic_1/M_AXIS_DATA
  S02_AXIS cic_2/M_AXIS_DATA
  S03_AXIS cic_3/M_AXIS_DATA
  S04_AXIS cic_4/M_AXIS_DATA
  S05_AXIS cic_5/M_AXIS_DATA
  S06_AXIS cic_6/M_AXIS_DATA
  S07_AXIS cic_7/M_AXIS_DATA
  S08_AXIS cic_8/M_AXIS_DATA
  S09_AXIS cic_9/M_AXIS_DATA
  S10_AXIS cic_10/M_AXIS_DATA
  S11_AXIS cic_11/M_AXIS_DATA
  S12_AXIS cic_12/M_AXIS_DATA
  S13_AXIS cic_13/M_AXIS_DATA
  S14_AXIS cic_14/M_AXIS_DATA
  S15_AXIS cic_15/M_AXIS_DATA
  aclk /pll_0/clk_out1
  aresetn /rst_0/peripheral_aresetn
}

# Create axis_dwidth_converter
cell xilinx.com:ip:axis_dwidth_converter conv_0 {
  S_TDATA_NUM_BYTES.VALUE_SRC USER
  S_TDATA_NUM_BYTES 64
  M_TDATA_NUM_BYTES 4
} {
  S_AXIS comb_0/M_AXIS
  aclk /pll_0/clk_out1
  aresetn /rst_0/peripheral_aresetn
}

# Create fir_compiler
cell xilinx.com:ip:fir_compiler fir_0 {
  DATA_WIDTH.VALUE_SRC USER
  DATA_WIDTH 32
  COEFFICIENTVECTOR {1.1961249863e-08, 1.2656113242e-08, 1.3309358908e-08, 1.3941034466e-08, 1.4577284871e-08, 1.5250466447e-08, 1.5999126380e-08, 1.6867833605e-08, 1.7906848440e-08, 1.9171620108e-08, 2.0722103492e-08, 2.2621888930e-08, 2.4937141679e-08, 2.7735350818e-08, 3.1083890736e-08, 3.5048401979e-08, 3.9691002072e-08, 4.5068340863e-08, 5.1229519040e-08, 5.8213892520e-08, 6.6048789471e-08, 7.4747170707e-08, 8.4305267918e-08, 9.4700237761e-08, 1.0588787301e-07, 1.1780041475e-07, 1.3034451197e-07, 1.4339937649e-07, 1.5681518263e-07, 1.7041176103e-07, 1.8397763614e-07, 1.9726945559e-07, 2.1001185770e-07, 2.2189782077e-07, 2.3258953374e-07, 2.4171982374e-07, 2.4889416998e-07, 2.5369332733e-07, 2.5567657596e-07, 2.5438560470e-07, 2.4934902788e-07, 2.4008752517e-07, 2.2611958469e-07, 2.0696781862e-07, 1.8216581031e-07, 1.5126544051e-07, 1.1384462961e-07, 6.9515421839e-08, 1.7932326771e-08, -4.1199176753e-08, -1.0811407849e-07, -1.8297932947e-07, -2.6588551026e-07, -3.5683880180e-07, -4.5575339146e-07, -5.6244445003e-07, -6.7662181787e-07, -7.9788453816e-07, -9.2571637381e-07, -1.0594824405e-06, -1.1984270824e-06, -1.3416731099e-06, -1.4882225061e-06, -1.6369586998e-06, -1.7866504848e-06, -1.9359576509e-06, -2.0834383713e-06, -2.2275583733e-06, -2.3667018929e-06, -2.4991843954e-06, -2.6232670130e-06, -2.7371726299e-06, -2.8391035153e-06, -2.9272603793e-06, -2.9998626976e-06, -3.0551701281e-06, -3.0915048107e-06, -3.1072743236e-06, -3.1009950394e-06, -3.0713156082e-06, -3.0170402731e-06, -2.9371517077e-06, -2.8308330538e-06, -2.6974888247e-06, -2.5367643381e-06, -2.3485633372e-06, -2.1330634646e-06, -1.8907292580e-06, -1.6223223537e-06, -1.3289085964e-06, -1.0118617792e-06, -6.7286376522e-07, -3.1390077269e-07, 6.2744355816e-08, 4.5450403794e-07, 8.5854208223e-07, 1.2717741070e-06, 1.6908922361e-06, 2.1123940033e-06, 2.5326153400e-06, 2.9477674568e-06, 3.3539773725e-06, 3.7473317769e-06, 4.1239238569e-06, 4.4799026514e-06, 4.8115244440e-06, 5.1152056481e-06, 5.3875765849e-06, 5.6255355116e-06, 5.8263022146e-06, 5.9874704478e-06, 6.1070584697e-06, 6.1835569145e-06, 6.2159732223e-06, 6.2038718500e-06, 6.1474094983e-06, 6.0473646055e-06, 5.9051603899e-06, 5.7228807660e-06, 5.5032785074e-06, 5.2497750941e-06, 4.9664517572e-06, 4.6580313120e-06, 4.3298504706e-06, 3.9878224217e-06, 3.6383895800e-06, 3.2884665245e-06, 2.9453732684e-06, 2.6167591349e-06, 2.3105176459e-06, 2.0346929637e-06, 1.7973785676e-06, 1.6066089745e-06, 1.4702454549e-06, 1.3958568136e-06, 1.3905964346e-06, 1.4610768981e-06, 1.6132435831e-06, 1.8522487606e-06, 2.1823277608e-06, 2.6066788603e-06, 3.1273485830e-06, 3.7451241349e-06, 4.4594347041e-06, 5.2682633429e-06, 6.1680711188e-06, 7.1537351635e-06, 8.2185021735e-06, 9.3539588131e-06, 1.0550020349e-05, 1.1794938703e-05, 1.3075330929e-05, 1.4376228960e-05, 1.5681151229e-05, 1.6972196563e-05, 1.8230160516e-05, 1.9434674022e-05, 2.0564364005e-05, 2.1597035305e-05, 2.2509872977e-05, 2.3279663760e-05, 2.3883035203e-05, 2.4296710670e-05, 2.4497778154e-05, 2.4463970577e-05, 2.4173954994e-05, 2.3607627882e-05, 2.2746413501e-05, 2.1573562086e-05, 2.0074444524e-05, 1.8236839983e-05, 1.6051212914e-05, 1.3510975758e-05, 1.0612733686e-05, 7.3565077152e-06, 3.7459326243e-06, -2.1157381935e-07, -4.5046736336e-06, -9.1180065209e-06, -1.4032125765e-05, -1.9223474710e-05, -2.4664406079e-05, -3.0323245996e-05, -3.6164404192e-05, -4.2148531408e-05, -4.8232724519e-05, -5.4370779428e-05, -6.0513491209e-05, -6.6609000452e-05, -7.2603184194e-05, -7.8440089244e-05, -8.4062405141e-05, -8.9411973420e-05, -9.4430329303e-05, -9.9059271400e-05, -1.0324145449e-04, -1.0692099996e-04, -1.1004411813e-04, -1.1255973607e-04, -1.1442012455e-04, -1.1558151707e-04, -1.1600471401e-04, -1.1565566460e-04, -1.1450601950e-04, -1.1253364661e-04, -1.0972310293e-04, -1.0606605545e-04, -1.0156164422e-04, -9.6216781265e-05, -9.0046379131e-05, -8.3073503774e-05, -7.5329446693e-05, -6.6853712149e-05, -5.7693915960e-05, -4.7905593178e-05, -3.7551912876e-05, -2.6703299179e-05, -1.5436958696e-05, -3.8363155200e-06, 8.0096439618e-06, 2.0007113125e-05, 3.2058289607e-05, 4.4062276739e-05, 5.5916055311e-05, 6.7515518031e-05, 7.8756558001e-05, 8.9536201588e-05, 9.9753775155e-05, 1.0931209429e-04, 1.1811866343e-04, 1.2608687319e-04, 1.3313718199e-04, 1.3919826855e-04, 1.4420814098e-04, 1.4811518866e-04, 1.5087916265e-04, 1.5247207066e-04, 1.5287897311e-04, 1.5209866690e-04, 1.5014424456e-04, 1.4704351683e-04, 1.4283928797e-04, 1.3758947409e-04, 1.3136705595e-04, 1.2425985922e-04, 1.1637015654e-04, 1.0781408765e-04, 9.8720895165e-05, 8.9231975840e-05, 7.9499748934e-05, 6.9686345180e-05, 5.9962122071e-05, 5.0504013152e-05, 4.1493721099e-05, 3.3115766424e-05, 2.5555405684e-05, 1.8996435016e-05, 1.3618896743e-05, 9.5967085478e-06, 7.0952364049e-06, 6.2688339697e-06, 7.2583724812e-06, 1.0188786407e-05, 1.5166661028e-05, 2.2277888902e-05, 3.1585422675e-05, 4.3127151932e-05, 5.6913931836e-05, 7.2927790966e-05, 9.1120345272e-05, 1.1141144420e-04, 1.3368807394e-04, 1.5780354126e-04, 1.8357695995e-04, 2.1079305951e-04, 2.3920233403e-04, 2.6852154618e-04, 2.9843459894e-04, 3.2859378451e-04, 3.5862141681e-04, 3.8811185051e-04, 4.1663388634e-04, 4.4373355840e-04, 4.6893729598e-04, 4.9175544814e-04, 5.1168615591e-04, 5.2821955296e-04, 5.4084227183e-04, 5.4904222921e-04, 5.5231366012e-04, 5.5016236740e-04, 5.4211114982e-04, 5.2770536879e-04, 5.0651861114e-04, 4.7815840291e-04, 4.4227192668e-04, 3.9855169348e-04, 3.4674111860e-04, 2.8663994967e-04, 2.1810949466e-04, 1.4107759738e-04, 5.5543308130e-05, -3.8418802029e-05, -1.4065473376e-04, -2.5092760749e-04, -3.6891471354e-04, -4.9420510855e-04, -6.2629779903e-04, -7.6460054973e-04, -9.0842935040e-04, -1.0570085703e-03, -1.2094718251e-03, -1.3648635759e-03, -1.5221414746e-03, -1.6801794646e-03, -1.8377716396e-03, -1.9936368570e-03, -2.1464240976e-03, -2.2947185558e-03, -2.4370484378e-03, -2.5718924414e-03, -2.6976878815e-03, -2.8128394227e-03, -2.9157283720e-03, -3.0047224798e-03, -3.0781861930e-03, -3.1344912968e-03, -3.1720278797e-03, -3.1892155494e-03, -3.1845148258e-03, -3.1564386330e-03, -3.1035638088e-03, -3.0245425496e-03, -2.9181137071e-03, -2.7831138490e-03, -2.6184880023e-03, -2.4232999904e-03, -2.1967422833e-03, -1.9381452771e-03, -1.6469859243e-03, -1.3228956388e-03, -9.6566740211e-04, -5.7526200496e-04, -1.5181335882e-04, 3.0436717758e-04, 7.9278750903e-04, 1.3127725959e-03, 1.8634633643e-03, 2.4438167652e-03, 3.0526070025e-03, 3.6884279453e-03, 4.3496967289e-03, 5.0346585412e-03, 5.7413925826e-03, 6.4678191796e-03, 7.2117080210e-03, 7.9706874816e-03, 8.7422549845e-03, 9.5237883505e-03, 1.0312558071e-02, 1.1105740435e-02, 1.1900431436e-02, 1.2693661374e-02, 1.3482410066e-02, 1.4263622567e-02, 1.5034225309e-02, 1.5791142555e-02, 1.6531313053e-02, 1.7251706798e-02, 1.7949341779e-02, 1.8621300604e-02, 1.9264746903e-02, 1.9876941379e-02, 2.0455257419e-02, 2.0997196151e-02, 2.1500400838e-02, 2.1962670528e-02, 2.2381972851e-02, 2.2756455881e-02, 2.3084458983e-02, 2.3364522569e-02, 2.3595396690e-02, 2.3776048416e-02, 2.3905667942e-02, 2.3983673382e-02, 2.4009714219e-02, 2.3983673382e-02, 2.3905667942e-02, 2.3776048416e-02, 2.3595396690e-02, 2.3364522569e-02, 2.3084458983e-02, 2.2756455881e-02, 2.2381972851e-02, 2.1962670528e-02, 2.1500400838e-02, 2.0997196151e-02, 2.0455257419e-02, 1.9876941379e-02, 1.9264746903e-02, 1.8621300604e-02, 1.7949341779e-02, 1.7251706798e-02, 1.6531313053e-02, 1.5791142555e-02, 1.5034225309e-02, 1.4263622567e-02, 1.3482410066e-02, 1.2693661374e-02, 1.1900431436e-02, 1.1105740435e-02, 1.0312558071e-02, 9.5237883505e-03, 8.7422549845e-03, 7.9706874816e-03, 7.2117080210e-03, 6.4678191796e-03, 5.7413925826e-03, 5.0346585412e-03, 4.3496967289e-03, 3.6884279453e-03, 3.0526070025e-03, 2.4438167652e-03, 1.8634633643e-03, 1.3127725959e-03, 7.9278750903e-04, 3.0436717758e-04, -1.5181335882e-04, -5.7526200496e-04, -9.6566740211e-04, -1.3228956388e-03, -1.6469859243e-03, -1.9381452771e-03, -2.1967422833e-03, -2.4232999904e-03, -2.6184880023e-03, -2.7831138490e-03, -2.9181137071e-03, -3.0245425496e-03, -3.1035638088e-03, -3.1564386330e-03, -3.1845148258e-03, -3.1892155494e-03, -3.1720278797e-03, -3.1344912968e-03, -3.0781861930e-03, -3.0047224798e-03, -2.9157283720e-03, -2.8128394227e-03, -2.6976878815e-03, -2.5718924414e-03, -2.4370484378e-03, -2.2947185558e-03, -2.1464240976e-03, -1.9936368570e-03, -1.8377716396e-03, -1.6801794646e-03, -1.5221414746e-03, -1.3648635759e-03, -1.2094718251e-03, -1.0570085703e-03, -9.0842935040e-04, -7.6460054973e-04, -6.2629779903e-04, -4.9420510855e-04, -3.6891471354e-04, -2.5092760749e-04, -1.4065473376e-04, -3.8418802029e-05, 5.5543308130e-05, 1.4107759738e-04, 2.1810949466e-04, 2.8663994967e-04, 3.4674111860e-04, 3.9855169348e-04, 4.4227192668e-04, 4.7815840291e-04, 5.0651861114e-04, 5.2770536879e-04, 5.4211114982e-04, 5.5016236740e-04, 5.5231366012e-04, 5.4904222921e-04, 5.4084227183e-04, 5.2821955296e-04, 5.1168615591e-04, 4.9175544814e-04, 4.6893729598e-04, 4.4373355840e-04, 4.1663388634e-04, 3.8811185051e-04, 3.5862141681e-04, 3.2859378451e-04, 2.9843459894e-04, 2.6852154618e-04, 2.3920233403e-04, 2.1079305951e-04, 1.8357695995e-04, 1.5780354126e-04, 1.3368807394e-04, 1.1141144420e-04, 9.1120345272e-05, 7.2927790966e-05, 5.6913931836e-05, 4.3127151932e-05, 3.1585422675e-05, 2.2277888902e-05, 1.5166661028e-05, 1.0188786407e-05, 7.2583724812e-06, 6.2688339697e-06, 7.0952364049e-06, 9.5967085478e-06, 1.3618896743e-05, 1.8996435016e-05, 2.5555405684e-05, 3.3115766424e-05, 4.1493721099e-05, 5.0504013152e-05, 5.9962122071e-05, 6.9686345180e-05, 7.9499748934e-05, 8.9231975840e-05, 9.8720895165e-05, 1.0781408765e-04, 1.1637015654e-04, 1.2425985922e-04, 1.3136705595e-04, 1.3758947409e-04, 1.4283928797e-04, 1.4704351683e-04, 1.5014424456e-04, 1.5209866690e-04, 1.5287897311e-04, 1.5247207066e-04, 1.5087916265e-04, 1.4811518866e-04, 1.4420814098e-04, 1.3919826855e-04, 1.3313718199e-04, 1.2608687319e-04, 1.1811866343e-04, 1.0931209429e-04, 9.9753775155e-05, 8.9536201588e-05, 7.8756558001e-05, 6.7515518031e-05, 5.5916055311e-05, 4.4062276739e-05, 3.2058289607e-05, 2.0007113125e-05, 8.0096439618e-06, -3.8363155200e-06, -1.5436958696e-05, -2.6703299179e-05, -3.7551912876e-05, -4.7905593178e-05, -5.7693915960e-05, -6.6853712149e-05, -7.5329446693e-05, -8.3073503774e-05, -9.0046379131e-05, -9.6216781265e-05, -1.0156164422e-04, -1.0606605545e-04, -1.0972310293e-04, -1.1253364661e-04, -1.1450601950e-04, -1.1565566460e-04, -1.1600471401e-04, -1.1558151707e-04, -1.1442012455e-04, -1.1255973607e-04, -1.1004411813e-04, -1.0692099996e-04, -1.0324145449e-04, -9.9059271400e-05, -9.4430329303e-05, -8.9411973420e-05, -8.4062405141e-05, -7.8440089244e-05, -7.2603184194e-05, -6.6609000452e-05, -6.0513491209e-05, -5.4370779428e-05, -4.8232724519e-05, -4.2148531408e-05, -3.6164404192e-05, -3.0323245996e-05, -2.4664406079e-05, -1.9223474710e-05, -1.4032125765e-05, -9.1180065209e-06, -4.5046736336e-06, -2.1157381935e-07, 3.7459326243e-06, 7.3565077152e-06, 1.0612733686e-05, 1.3510975758e-05, 1.6051212914e-05, 1.8236839983e-05, 2.0074444524e-05, 2.1573562086e-05, 2.2746413501e-05, 2.3607627882e-05, 2.4173954994e-05, 2.4463970577e-05, 2.4497778154e-05, 2.4296710670e-05, 2.3883035203e-05, 2.3279663760e-05, 2.2509872977e-05, 2.1597035305e-05, 2.0564364005e-05, 1.9434674022e-05, 1.8230160516e-05, 1.6972196563e-05, 1.5681151229e-05, 1.4376228960e-05, 1.3075330929e-05, 1.1794938703e-05, 1.0550020349e-05, 9.3539588131e-06, 8.2185021735e-06, 7.1537351635e-06, 6.1680711188e-06, 5.2682633429e-06, 4.4594347041e-06, 3.7451241349e-06, 3.1273485830e-06, 2.6066788603e-06, 2.1823277608e-06, 1.8522487606e-06, 1.6132435831e-06, 1.4610768981e-06, 1.3905964346e-06, 1.3958568136e-06, 1.4702454549e-06, 1.6066089745e-06, 1.7973785676e-06, 2.0346929637e-06, 2.3105176459e-06, 2.6167591349e-06, 2.9453732684e-06, 3.2884665245e-06, 3.6383895800e-06, 3.9878224217e-06, 4.3298504706e-06, 4.6580313120e-06, 4.9664517572e-06, 5.2497750941e-06, 5.5032785074e-06, 5.7228807660e-06, 5.9051603899e-06, 6.0473646055e-06, 6.1474094983e-06, 6.2038718500e-06, 6.2159732223e-06, 6.1835569145e-06, 6.1070584697e-06, 5.9874704478e-06, 5.8263022146e-06, 5.6255355116e-06, 5.3875765849e-06, 5.1152056481e-06, 4.8115244440e-06, 4.4799026514e-06, 4.1239238569e-06, 3.7473317769e-06, 3.3539773725e-06, 2.9477674568e-06, 2.5326153400e-06, 2.1123940033e-06, 1.6908922361e-06, 1.2717741070e-06, 8.5854208223e-07, 4.5450403794e-07, 6.2744355816e-08, -3.1390077269e-07, -6.7286376522e-07, -1.0118617792e-06, -1.3289085964e-06, -1.6223223537e-06, -1.8907292580e-06, -2.1330634646e-06, -2.3485633372e-06, -2.5367643381e-06, -2.6974888247e-06, -2.8308330538e-06, -2.9371517077e-06, -3.0170402731e-06, -3.0713156082e-06, -3.1009950394e-06, -3.1072743236e-06, -3.0915048107e-06, -3.0551701281e-06, -2.9998626976e-06, -2.9272603793e-06, -2.8391035153e-06, -2.7371726299e-06, -2.6232670130e-06, -2.4991843954e-06, -2.3667018929e-06, -2.2275583733e-06, -2.0834383713e-06, -1.9359576509e-06, -1.7866504848e-06, -1.6369586998e-06, -1.4882225061e-06, -1.3416731099e-06, -1.1984270824e-06, -1.0594824405e-06, -9.2571637381e-07, -7.9788453816e-07, -6.7662181787e-07, -5.6244445003e-07, -4.5575339146e-07, -3.5683880180e-07, -2.6588551026e-07, -1.8297932947e-07, -1.0811407849e-07, -4.1199176753e-08, 1.7932326771e-08, 6.9515421839e-08, 1.1384462961e-07, 1.5126544051e-07, 1.8216581031e-07, 2.0696781862e-07, 2.2611958469e-07, 2.4008752517e-07, 2.4934902788e-07, 2.5438560470e-07, 2.5567657596e-07, 2.5369332733e-07, 2.4889416998e-07, 2.4171982374e-07, 2.3258953374e-07, 2.2189782077e-07, 2.1001185770e-07, 1.9726945559e-07, 1.8397763614e-07, 1.7041176103e-07, 1.5681518263e-07, 1.4339937649e-07, 1.3034451197e-07, 1.1780041475e-07, 1.0588787301e-07, 9.4700237761e-08, 8.4305267918e-08, 7.4747170707e-08, 6.6048789471e-08, 5.8213892520e-08, 5.1229519040e-08, 4.5068340863e-08, 3.9691002072e-08, 3.5048401979e-08, 3.1083890736e-08, 2.7735350818e-08, 2.4937141679e-08, 2.2621888930e-08, 2.0722103492e-08, 1.9171620108e-08, 1.7906848440e-08, 1.6867833605e-08, 1.5999126380e-08, 1.5250466447e-08, 1.4577284871e-08, 1.3941034466e-08, 1.3309358908e-08, 1.2656113242e-08, 1.1961249863e-08}
  COEFFICIENT_WIDTH 24
  QUANTIZATION Quantize_Only
  BESTPRECISION true
  FILTER_TYPE Decimation
  RATE_CHANGE_TYPE Fixed_Fractional
  INTERPOLATION_RATE 24
  DECIMATION_RATE 25
  NUMBER_CHANNELS 16
  NUMBER_PATHS 1
  SAMPLE_FREQUENCY 0.5
  CLOCK_FREQUENCY 125
  OUTPUT_ROUNDING_MODE Convergent_Rounding_to_Even
  OUTPUT_WIDTH 25
  HAS_ARESETN true
} {
  S_AXIS_DATA conv_0/M_AXIS
  aclk /pll_0/clk_out1
  aresetn /rst_0/peripheral_aresetn
}

# Create axis_subset_converter
cell xilinx.com:ip:axis_subset_converter subset_0 {
  S_TDATA_NUM_BYTES.VALUE_SRC USER
  M_TDATA_NUM_BYTES.VALUE_SRC USER
  S_TDATA_NUM_BYTES 4
  M_TDATA_NUM_BYTES 3
  TDATA_REMAP {tdata[23:0]}
} {
  S_AXIS fir_0/M_AXIS_DATA
  aclk /pll_0/clk_out1
  aresetn /rst_0/peripheral_aresetn
}

# Create fir_compiler
cell xilinx.com:ip:fir_compiler fir_1 {
  DATA_WIDTH.VALUE_SRC USER
  DATA_WIDTH 24
  COEFFICIENTVECTOR {3.1509902550e-09, -6.7476788504e-09, -1.6133003296e-08, -2.2415990522e-08, -2.3717726039e-08, -1.9291793436e-08, -9.6170736920e-09, 3.7553594387e-09, 1.8453734462e-08, 3.1533736102e-08, 3.9743825633e-08, 3.9942129690e-08, 2.9810578847e-08, 8.8710259750e-09, -2.0450381408e-08, -5.2219241860e-08, -7.7446359845e-08, -8.5989683387e-08, -6.9782205041e-08, -2.6584640735e-08, 3.7108513502e-08, 1.0553293445e-07, 1.5653606233e-07, 1.6783580618e-07, 1.2520884032e-07, 2.9948780953e-08, -9.7517683988e-08, -2.2027419746e-07, -2.9439145149e-07, -2.8328837380e-07, -1.7268144520e-07, 1.9374717962e-08, 2.4218444969e-07, 4.2400690316e-07, 4.9367152378e-07, 4.0620612125e-07, 1.6381806850e-07, -1.7632380762e-07, -5.1273275903e-07, -7.2803785551e-07, -7.2793713216e-07, -4.7833566549e-07, -2.6980960656e-08, 5.0064159760e-07, 9.3366068124e-07, 1.1091061444e-06, 9.3019595082e-07, 4.0958376907e-07, -3.1991945978e-07, -1.0354290851e-06, -1.4897203487e-06, -1.4940854882e-06, -9.9130511529e-07, -9.1803988106e-08, 9.4425549149e-07, 1.7791914237e-06, 2.1054637914e-06, 1.7552379285e-06, 7.7381313204e-07, -5.7269503284e-07, -1.8652424111e-06, -2.6597601909e-06, -2.6369277563e-06, -1.7241170379e-06, -1.4449763317e-07, 1.6308123822e-06, 3.0203142991e-06, 3.5203679059e-06, 2.8838809346e-06, 1.2270765333e-06, -9.7869946219e-07, -3.0372042429e-06, -4.2451340338e-06, -4.1301018731e-06, -2.6288818327e-06, -1.4025274410e-07, 2.5752317426e-06, 4.6257773351e-06, 5.2800585836e-06, 4.2250399246e-06, 1.6997676116e-06, -1.5487642811e-06, -4.4843725846e-06, -6.1119419028e-06, -5.8140420514e-06, -3.5801600651e-06, -4.4607107580e-08, 3.6905736926e-06, 6.3995879822e-06, 7.1363445357e-06, 5.5641706723e-06, 2.0968943993e-06, -2.2052035193e-06, -5.9606273373e-06, -7.9131778271e-06, -7.3545110199e-06, -4.3797078689e-06, 1.1626239721e-07, 4.7091268662e-06, 7.9009646174e-06, 8.6138167769e-06, 6.5568685985e-06, 2.3408522538e-06, -2.7047005246e-06, -6.9565914200e-06, -9.0278583399e-06, -8.2269641509e-06, -4.7917991128e-06, 1.8216502810e-07, 5.0980188993e-06, 8.3803057882e-06, 8.9903029709e-06, 6.7615824466e-06, 2.4446389613e-06, -2.5518128976e-06, -6.6325768887e-06, -8.5393711642e-06, -7.7495087761e-06, -4.6193595415e-06, -2.3219048974e-07, 3.9878189303e-06, 6.7554143769e-06, 7.3309700074e-06, 5.7196433252e-06, 2.6075213615e-06, -9.2518564426e-07, -3.7970650856e-06, -5.2772700397e-06, -5.1744710310e-06, -3.8112344236e-06, -1.8178241823e-06, 1.5302226942e-07, 1.6657206403e-06, 2.6007545811e-06, 3.0864483034e-06, 3.3153482800e-06, 3.3506473381e-06, 3.0330210902e-06, 2.0505219506e-06, 1.5281801750e-07, -2.5887886207e-06, -5.6152528706e-06, -7.9262608653e-06, -8.3765591040e-06, -6.1462681144e-06, -1.2105747028e-06, 5.4041558745e-06, 1.1742437156e-05, 1.5456164588e-05, 1.4624869114e-05, 8.5866392560e-06, -1.5489049355e-06, -1.2966884586e-05, -2.1871916814e-05, -2.4708504890e-05, -1.9497931750e-05, -6.8094156693e-06, 1.0051339344e-05, 2.5821200750e-05, 3.4878425272e-05, 3.3191337436e-05, 1.9957070047e-05, -1.7006243558e-06, -2.5342425744e-05, -4.3073108121e-05, -4.8133698414e-05, -3.7389065754e-05, -1.2818607917e-05, 1.8623823066e-05, 4.6902840908e-05, 6.2062028138e-05, 5.7671287862e-05, 3.3383935925e-05, -4.3729052992e-06, -4.3987813091e-05, -7.2188225533e-05, -7.8445847631e-05, -5.8814824912e-05, -1.7827954410e-05, 3.2363378026e-05, 7.5530576487e-05, 9.6582048484e-05, 8.6786103334e-05, 4.7209914333e-05, -1.0935593935e-05, -6.9384807016e-05, -1.0852401893e-04, -1.1394430400e-04, -8.1659903228e-05, -2.0124957667e-05, 5.1843052845e-05, 1.1078145701e-04, 1.3621889018e-04, 1.1774894689e-04, 5.9106406634e-05, -2.2306612407e-05, -1.0053508341e-04, -1.4934324785e-04, -1.5100994605e-04, -1.0269873265e-04, -1.8129048489e-05, 7.6243157091e-05, 1.4950398990e-04, 1.7643005578e-04, 1.4616817057e-04, 6.6458848141e-05, -3.8203822583e-05, -1.3410042054e-04, -1.8919135465e-04, -1.8385949819e-04, -1.1787497259e-04, -1.1112038069e-05, 1.0243060487e-04, 1.8551343114e-04, 2.0994313991e-04, 1.6618513413e-04, 6.7006899434e-05, -5.6223576519e-05, -1.6352529435e-04, -2.1937945575e-04, -2.0458371204e-04, -1.2289599145e-04, -1.5120943056e-07, 1.2397841733e-04, 2.0893439590e-04, 2.2666757461e-04, 1.7100685384e-04, 5.9832330765e-05, -7.0727572050e-05, -1.7818039118e-04, -2.2768516941e-04, -2.0351746984e-04, -1.1415232018e-04, 1.0716041975e-05, 1.3017742694e-04, 2.0572801569e-04, 2.1390985113e-04, 1.5374111758e-04, 4.6524417400e-05, -7.1760429945e-05, -1.6275669598e-04, -1.9845769492e-04, -1.7004501516e-04, -8.9965008759e-05, 1.3223166091e-05, 1.0518510130e-04, 1.5756454331e-04, 1.5703318716e-04, 1.0863735696e-04, 3.2593541784e-05, -4.3936658578e-05, -9.6828932576e-05, -1.1300557547e-04, -9.3678348933e-05, -5.1982545771e-05, -6.3562837990e-06, 2.7392349956e-05, 4.2016571359e-05, 4.0422561073e-05, 3.2501048125e-05, 2.8852112575e-05, 3.4426606937e-05, 4.5183838914e-05, 4.9574605514e-05, 3.4495630588e-05, -6.7884038911e-06, -6.8461527597e-05, -1.3107269706e-04, -1.6702354675e-04, -1.5132231748e-04, -7.3723474374e-05, 5.2881906829e-05, 1.9220165539e-04, 2.9406806558e-04, 3.1161390385e-04, 2.2047139381e-04, 3.2643947527e-05, -2.0133513084e-04, -4.0471761684e-04, -4.9887525737e-04, -4.3221535288e-04, -2.0342961436e-04, 1.3045489654e-04, 4.6634700533e-04, 6.8591955843e-04, 6.9613593430e-04, 4.6593311600e-04, 4.5257862551e-05, -4.4258665024e-04, -8.3572272680e-04, -9.8631317910e-04, -8.1425898391e-04, -3.4299076971e-04, 2.9730383786e-04, 9.0365265971e-04, 1.2634012934e-03, 1.2272498881e-03, 7.6759441213e-04, 2.7624061943e-07, -8.4144304099e-04, -1.4762033759e-03, -1.6671318374e-03, -1.3076849472e-03, -4.7034427050e-04, 6.0206727740e-04, 1.5645680863e-03, 2.0793898658e-03, 1.9323937135e-03, 1.1170313349e-03, -1.4578745874e-04, -1.4641177273e-03, -2.3946430165e-03, -2.5897278478e-03, -1.9237083743e-03, -5.5354090039e-04, 1.1121377565e-03, 2.5321754773e-03, 3.2065096088e-03, 2.8492297462e-03, 1.5021033389e-03, -4.5460268015e-04, -2.4054331390e-03, -3.6906283467e-03, -3.8262738365e-03, -2.6811743028e-03, -5.4729480834e-04, 1.9278767001e-03, 3.9341775294e-03, 4.7606259944e-03, 4.0427006526e-03, 1.9104273514e-03, -1.0198311405e-03, -3.8182489488e-03, -5.5324637111e-03, -5.5068547373e-03, -3.6246165868e-03, -3.8566682174e-04, 3.2174245258e-03, 5.9980296388e-03, 6.9605688447e-03, 5.6489116264e-03, 2.3385429359e-03, -2.0030417063e-03, -5.9909218542e-03, -8.2571124399e-03, -7.9107579721e-03, -4.8721032820e-03, 4.0496525415e-05, 5.3175353892e-03, 9.2117387334e-03, 1.0304885886e-02, 8.0077168056e-03, 2.8215833016e-03, -3.7431311060e-03, -9.5909517794e-03, -1.2693478285e-02, -1.1771649058e-02, -6.7848934196e-03, 9.4979525994e-04, 9.0782056893e-03, 1.4899594352e-02, 1.6232102400e-02, 1.2184472840e-02, 3.5739608761e-03, -7.1768716669e-03, -1.6677938921e-02, -2.1587023157e-02, -1.9711592906e-02, -1.0852866917e-02, 2.9012218001e-03, 1.7581078778e-02, 2.8381820198e-02, 3.1091984515e-02, 2.3506475102e-02, 6.3960771416e-03, -1.6297980988e-02, -3.8174834018e-02, -5.1867551926e-02, -5.0963163700e-02, -3.1847399586e-02, 5.0383592722e-03, 5.4907328724e-02, 1.0943903973e-01, 1.5853465364e-01, 1.9259916623e-01, 2.0478119186e-01, 1.9259916623e-01, 1.5853465364e-01, 1.0943903973e-01, 5.4907328724e-02, 5.0383592722e-03, -3.1847399586e-02, -5.0963163700e-02, -5.1867551926e-02, -3.8174834018e-02, -1.6297980988e-02, 6.3960771416e-03, 2.3506475102e-02, 3.1091984515e-02, 2.8381820198e-02, 1.7581078778e-02, 2.9012218001e-03, -1.0852866917e-02, -1.9711592906e-02, -2.1587023157e-02, -1.6677938921e-02, -7.1768716669e-03, 3.5739608761e-03, 1.2184472840e-02, 1.6232102400e-02, 1.4899594352e-02, 9.0782056893e-03, 9.4979525994e-04, -6.7848934196e-03, -1.1771649058e-02, -1.2693478285e-02, -9.5909517794e-03, -3.7431311060e-03, 2.8215833016e-03, 8.0077168056e-03, 1.0304885886e-02, 9.2117387334e-03, 5.3175353892e-03, 4.0496525415e-05, -4.8721032820e-03, -7.9107579721e-03, -8.2571124399e-03, -5.9909218542e-03, -2.0030417063e-03, 2.3385429359e-03, 5.6489116264e-03, 6.9605688447e-03, 5.9980296388e-03, 3.2174245258e-03, -3.8566682174e-04, -3.6246165868e-03, -5.5068547373e-03, -5.5324637111e-03, -3.8182489488e-03, -1.0198311405e-03, 1.9104273514e-03, 4.0427006526e-03, 4.7606259944e-03, 3.9341775294e-03, 1.9278767001e-03, -5.4729480834e-04, -2.6811743028e-03, -3.8262738365e-03, -3.6906283467e-03, -2.4054331390e-03, -4.5460268015e-04, 1.5021033389e-03, 2.8492297462e-03, 3.2065096088e-03, 2.5321754773e-03, 1.1121377565e-03, -5.5354090039e-04, -1.9237083743e-03, -2.5897278478e-03, -2.3946430165e-03, -1.4641177273e-03, -1.4578745874e-04, 1.1170313349e-03, 1.9323937135e-03, 2.0793898658e-03, 1.5645680863e-03, 6.0206727740e-04, -4.7034427050e-04, -1.3076849472e-03, -1.6671318374e-03, -1.4762033759e-03, -8.4144304099e-04, 2.7624061943e-07, 7.6759441213e-04, 1.2272498881e-03, 1.2634012934e-03, 9.0365265971e-04, 2.9730383786e-04, -3.4299076971e-04, -8.1425898391e-04, -9.8631317910e-04, -8.3572272680e-04, -4.4258665024e-04, 4.5257862551e-05, 4.6593311600e-04, 6.9613593430e-04, 6.8591955843e-04, 4.6634700533e-04, 1.3045489654e-04, -2.0342961436e-04, -4.3221535288e-04, -4.9887525737e-04, -4.0471761684e-04, -2.0133513084e-04, 3.2643947527e-05, 2.2047139381e-04, 3.1161390385e-04, 2.9406806558e-04, 1.9220165539e-04, 5.2881906829e-05, -7.3723474374e-05, -1.5132231748e-04, -1.6702354675e-04, -1.3107269706e-04, -6.8461527597e-05, -6.7884038911e-06, 3.4495630588e-05, 4.9574605514e-05, 4.5183838914e-05, 3.4426606937e-05, 2.8852112575e-05, 3.2501048125e-05, 4.0422561073e-05, 4.2016571359e-05, 2.7392349956e-05, -6.3562837990e-06, -5.1982545771e-05, -9.3678348933e-05, -1.1300557547e-04, -9.6828932576e-05, -4.3936658578e-05, 3.2593541784e-05, 1.0863735696e-04, 1.5703318716e-04, 1.5756454331e-04, 1.0518510130e-04, 1.3223166091e-05, -8.9965008759e-05, -1.7004501516e-04, -1.9845769492e-04, -1.6275669598e-04, -7.1760429945e-05, 4.6524417400e-05, 1.5374111758e-04, 2.1390985113e-04, 2.0572801569e-04, 1.3017742694e-04, 1.0716041975e-05, -1.1415232018e-04, -2.0351746984e-04, -2.2768516941e-04, -1.7818039118e-04, -7.0727572050e-05, 5.9832330765e-05, 1.7100685384e-04, 2.2666757461e-04, 2.0893439590e-04, 1.2397841733e-04, -1.5120943056e-07, -1.2289599145e-04, -2.0458371204e-04, -2.1937945575e-04, -1.6352529435e-04, -5.6223576519e-05, 6.7006899434e-05, 1.6618513413e-04, 2.0994313991e-04, 1.8551343114e-04, 1.0243060487e-04, -1.1112038069e-05, -1.1787497259e-04, -1.8385949819e-04, -1.8919135465e-04, -1.3410042054e-04, -3.8203822583e-05, 6.6458848141e-05, 1.4616817057e-04, 1.7643005578e-04, 1.4950398990e-04, 7.6243157091e-05, -1.8129048489e-05, -1.0269873265e-04, -1.5100994605e-04, -1.4934324785e-04, -1.0053508341e-04, -2.2306612407e-05, 5.9106406634e-05, 1.1774894689e-04, 1.3621889018e-04, 1.1078145701e-04, 5.1843052845e-05, -2.0124957667e-05, -8.1659903228e-05, -1.1394430400e-04, -1.0852401893e-04, -6.9384807016e-05, -1.0935593935e-05, 4.7209914333e-05, 8.6786103334e-05, 9.6582048484e-05, 7.5530576487e-05, 3.2363378026e-05, -1.7827954410e-05, -5.8814824912e-05, -7.8445847631e-05, -7.2188225533e-05, -4.3987813091e-05, -4.3729052992e-06, 3.3383935925e-05, 5.7671287862e-05, 6.2062028138e-05, 4.6902840908e-05, 1.8623823066e-05, -1.2818607917e-05, -3.7389065754e-05, -4.8133698414e-05, -4.3073108121e-05, -2.5342425744e-05, -1.7006243558e-06, 1.9957070047e-05, 3.3191337436e-05, 3.4878425272e-05, 2.5821200750e-05, 1.0051339344e-05, -6.8094156693e-06, -1.9497931750e-05, -2.4708504890e-05, -2.1871916814e-05, -1.2966884586e-05, -1.5489049355e-06, 8.5866392560e-06, 1.4624869114e-05, 1.5456164588e-05, 1.1742437156e-05, 5.4041558745e-06, -1.2105747028e-06, -6.1462681144e-06, -8.3765591040e-06, -7.9262608653e-06, -5.6152528706e-06, -2.5887886207e-06, 1.5281801750e-07, 2.0505219506e-06, 3.0330210902e-06, 3.3506473381e-06, 3.3153482800e-06, 3.0864483034e-06, 2.6007545811e-06, 1.6657206403e-06, 1.5302226942e-07, -1.8178241823e-06, -3.8112344236e-06, -5.1744710310e-06, -5.2772700397e-06, -3.7970650856e-06, -9.2518564426e-07, 2.6075213615e-06, 5.7196433252e-06, 7.3309700074e-06, 6.7554143769e-06, 3.9878189303e-06, -2.3219048974e-07, -4.6193595415e-06, -7.7495087761e-06, -8.5393711642e-06, -6.6325768887e-06, -2.5518128976e-06, 2.4446389613e-06, 6.7615824466e-06, 8.9903029709e-06, 8.3803057882e-06, 5.0980188993e-06, 1.8216502810e-07, -4.7917991128e-06, -8.2269641509e-06, -9.0278583399e-06, -6.9565914200e-06, -2.7047005246e-06, 2.3408522538e-06, 6.5568685985e-06, 8.6138167769e-06, 7.9009646174e-06, 4.7091268662e-06, 1.1626239721e-07, -4.3797078689e-06, -7.3545110199e-06, -7.9131778271e-06, -5.9606273373e-06, -2.2052035193e-06, 2.0968943993e-06, 5.5641706723e-06, 7.1363445357e-06, 6.3995879822e-06, 3.6905736926e-06, -4.4607107580e-08, -3.5801600651e-06, -5.8140420514e-06, -6.1119419028e-06, -4.4843725846e-06, -1.5487642811e-06, 1.6997676116e-06, 4.2250399246e-06, 5.2800585836e-06, 4.6257773351e-06, 2.5752317426e-06, -1.4025274410e-07, -2.6288818327e-06, -4.1301018731e-06, -4.2451340338e-06, -3.0372042429e-06, -9.7869946219e-07, 1.2270765333e-06, 2.8838809346e-06, 3.5203679059e-06, 3.0203142991e-06, 1.6308123822e-06, -1.4449763317e-07, -1.7241170379e-06, -2.6369277563e-06, -2.6597601909e-06, -1.8652424111e-06, -5.7269503284e-07, 7.7381313204e-07, 1.7552379285e-06, 2.1054637914e-06, 1.7791914237e-06, 9.4425549149e-07, -9.1803988106e-08, -9.9130511529e-07, -1.4940854882e-06, -1.4897203487e-06, -1.0354290851e-06, -3.1991945978e-07, 4.0958376907e-07, 9.3019595082e-07, 1.1091061444e-06, 9.3366068124e-07, 5.0064159760e-07, -2.6980960656e-08, -4.7833566549e-07, -7.2793713216e-07, -7.2803785551e-07, -5.1273275903e-07, -1.7632380762e-07, 1.6381806850e-07, 4.0620612125e-07, 4.9367152378e-07, 4.2400690316e-07, 2.4218444969e-07, 1.9374717962e-08, -1.7268144520e-07, -2.8328837380e-07, -2.9439145149e-07, -2.2027419746e-07, -9.7517683988e-08, 2.9948780953e-08, 1.2520884032e-07, 1.6783580618e-07, 1.5653606233e-07, 1.0553293445e-07, 3.7108513502e-08, -2.6584640735e-08, -6.9782205041e-08, -8.5989683387e-08, -7.7446359845e-08, -5.2219241860e-08, -2.0450381408e-08, 8.8710259750e-09, 2.9810578847e-08, 3.9942129690e-08, 3.9743825633e-08, 3.1533736102e-08, 1.8453734462e-08, 3.7553594387e-09, -9.6170736920e-09, -1.9291793436e-08, -2.3717726039e-08, -2.2415990522e-08, -1.6133003296e-08, -6.7476788504e-09, 3.1509902550e-09}
  COEFFICIENT_WIDTH 24
  QUANTIZATION Quantize_Only
  BESTPRECISION true
  FILTER_TYPE Decimation
  RATE_CHANGE_TYPE Fixed_Fractional
  INTERPOLATION_RATE 2
  DECIMATION_RATE 5
  NUMBER_CHANNELS 16
  NUMBER_PATHS 1
  SAMPLE_FREQUENCY 0.48
  CLOCK_FREQUENCY 125
  OUTPUT_ROUNDING_MODE Convergent_Rounding_to_Even
  OUTPUT_WIDTH 26
  HAS_ARESETN true
} {
  S_AXIS_DATA subset_0/M_AXIS
  aclk /pll_0/clk_out1
  aresetn /rst_0/peripheral_aresetn
}

# Create axis_dwidth_converter
cell xilinx.com:ip:axis_dwidth_converter conv_1 {
  S_TDATA_NUM_BYTES.VALUE_SRC USER
  S_TDATA_NUM_BYTES 4
  M_TDATA_NUM_BYTES 64
} {
  S_AXIS fir_1/M_AXIS_DATA
  aclk /pll_0/clk_out1
  aresetn /rst_0/peripheral_aresetn
}

# Create axis_subset_converter
cell xilinx.com:ip:axis_subset_converter subset_1 {
  S_TDATA_NUM_BYTES.VALUE_SRC USER
  M_TDATA_NUM_BYTES.VALUE_SRC USER
  S_TDATA_NUM_BYTES 64
  M_TDATA_NUM_BYTES 48
  TDATA_REMAP {tdata[455:448],tdata[463:456],tdata[471:464],tdata[487:480],tdata[495:488],tdata[503:496],tdata[391:384],tdata[399:392],tdata[407:400],tdata[423:416],tdata[431:424],tdata[439:432],tdata[327:320],tdata[335:328],tdata[343:336],tdata[359:352],tdata[367:360],tdata[375:368],tdata[263:256],tdata[271:264],tdata[279:272],tdata[295:288],tdata[303:296],tdata[311:304],tdata[199:192],tdata[207:200],tdata[215:208],tdata[231:224],tdata[239:232],tdata[247:240],tdata[135:128],tdata[143:136],tdata[151:144],tdata[167:160],tdata[175:168],tdata[183:176],tdata[71:64],tdata[79:72],tdata[87:80],tdata[103:96],tdata[111:104],tdata[119:112],tdata[7:0],tdata[15:8],tdata[23:16],tdata[39:32],tdata[47:40],tdata[55:48]}
} {
  S_AXIS conv_1/M_AXIS
  aclk /pll_0/clk_out1
  aresetn /rst_0/peripheral_aresetn
}

# Create axis_fifo
cell pavel-demin:user:axis_fifo fifo_0 {
  S_AXIS_TDATA_WIDTH 384
  M_AXIS_TDATA_WIDTH 384
  WRITE_DEPTH 1024
  ALWAYS_READY TRUE
} {
  S_AXIS subset_1/M_AXIS
  read_count hub_0/sts_data
  aclk /pll_0/clk_out1
  aresetn slice_0/dout
}

# Create axis_dwidth_converter
cell xilinx.com:ip:axis_dwidth_converter conv_2 {
  S_TDATA_NUM_BYTES.VALUE_SRC USER
  S_TDATA_NUM_BYTES 48
  M_TDATA_NUM_BYTES 4
} {
  S_AXIS fifo_0/M_AXIS
  M_AXIS hub_0/S00_AXIS
  aclk /pll_0/clk_out1
  aresetn slice_0/dout
}
