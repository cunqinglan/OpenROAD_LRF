// Benchmark "c432" written by ABC on Mon Feb  9 15:03:24 2026

module c432 ( 
    N1, N4, N8, N11, N14, N17, N21, N24, N27, N30, N34, N37, N40, N43, N47,
    N50, N53, N56, N60, N63, N66, N69, N73, N76, N79, N82, N86, N89, N92,
    N95, N99, N102, N105, N108, N112, N115,
    N223, N329, N370, N421, N430, N431, N432  );
  input  N1, N4, N8, N11, N14, N17, N21, N24, N27, N30, N34, N37, N40,
    N43, N47, N50, N53, N56, N60, N63, N66, N69, N73, N76, N79, N82, N86,
    N89, N92, N95, N99, N102, N105, N108, N112, N115;
  output N223, N329, N370, N421, N430, N431, N432;
  wire new_n44, new_n45, new_n46, new_n47, new_n48, new_n49, new_n50,
    new_n51, new_n52, new_n53, new_n54, new_n55, new_n56, new_n57, new_n58,
    new_n59, new_n60, new_n61, new_n62, new_n63, new_n64, new_n66, new_n67,
    new_n68, new_n69, new_n70, new_n71, new_n72, new_n73, new_n74, new_n75,
    new_n76, new_n77, new_n78, new_n79, new_n80, new_n81, new_n82, new_n83,
    new_n84, new_n85, new_n86, new_n87, new_n88, new_n89, new_n90, new_n91,
    new_n92, new_n93, new_n94, new_n95, new_n96, new_n97, new_n98, new_n99,
    new_n100, new_n101, new_n102, new_n103, new_n104, new_n105, new_n106,
    new_n107, new_n108, new_n109, new_n110, new_n111, new_n112, new_n113,
    new_n114, new_n115, new_n116, new_n117, new_n118, new_n119, new_n120,
    new_n121, new_n122, new_n123, new_n124, new_n125, new_n126, new_n127,
    new_n129, new_n130, new_n131, new_n132, new_n133, new_n134, new_n135,
    new_n136, new_n137, new_n138, new_n139, new_n140, new_n141, new_n142,
    new_n143, new_n144, new_n145, new_n146, new_n147, new_n148, new_n149,
    new_n150, new_n151, new_n152, new_n153, new_n154, new_n155, new_n156,
    new_n157, new_n158, new_n159, new_n160, new_n161, new_n162, new_n163,
    new_n164, new_n165, new_n166, new_n167, new_n168, new_n169, new_n170,
    new_n171, new_n172, new_n173, new_n174, new_n175, new_n176, new_n177,
    new_n178, new_n179, new_n180, new_n181, new_n182, new_n183, new_n184,
    new_n186, new_n187, new_n188, new_n189, new_n190, new_n191, new_n192,
    new_n193, new_n194, new_n195, new_n196, new_n197, new_n198, new_n199,
    new_n200, new_n201, new_n202, new_n203, new_n204, new_n205, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n217, new_n218, new_n219, new_n220, new_n221,
    new_n223, new_n224, new_n225, new_n226, new_n228, new_n229, new_n230;
  INVx1_ASAP7_75t_L            g000(.A(N108), .Y(new_n44));
  NOR2xp33_ASAP7_75t_SRAM      g001(.A(N102), .B(new_n44), .Y(new_n45));
  INVx1_ASAP7_75t_L            g002(.A(new_n45), .Y(new_n46));
  INVx1_ASAP7_75t_L            g003(.A(N89), .Y(new_n47));
  NAND2xp33_ASAP7_75t_SRAM     g004(.A(N95), .B(new_n47), .Y(new_n48));
  INVx1_ASAP7_75t_L            g005(.A(new_n48), .Y(new_n49));
  INVx1_ASAP7_75t_L            g006(.A(N82), .Y(new_n50));
  NOR2xp33_ASAP7_75t_SRAM      g007(.A(N76), .B(new_n50), .Y(new_n51));
  INVx1_ASAP7_75t_L            g008(.A(N63), .Y(new_n52));
  NAND2xp33_ASAP7_75t_SRAM     g009(.A(N69), .B(new_n52), .Y(new_n53));
  INVx1_ASAP7_75t_L            g010(.A(N50), .Y(new_n54));
  NAND2xp33_ASAP7_75t_SRAM     g011(.A(N56), .B(new_n54), .Y(new_n55));
  INVx1_ASAP7_75t_L            g012(.A(N37), .Y(new_n56));
  NAND2xp33_ASAP7_75t_SRAM     g013(.A(N43), .B(new_n56), .Y(new_n57));
  INVx1_ASAP7_75t_L            g014(.A(N24), .Y(new_n58));
  NAND2xp33_ASAP7_75t_SRAM     g015(.A(N30), .B(new_n58), .Y(new_n59));
  INVx1_ASAP7_75t_L            g016(.A(N1), .Y(new_n60));
  INVx1_ASAP7_75t_L            g017(.A(N11), .Y(new_n61));
  AOI22xp33_ASAP7_75t_SRAM     g018(.A1(new_n60), .A2(N4), .B1(new_n61), .B2(N17), .Y(new_n62));
  NAND5xp2_ASAP7_75t_SRAM      g019(.A(new_n53), .B(new_n62), .C(new_n55), .D(new_n57), .E(new_n59), .Y(new_n63));
  NOR3xp33_ASAP7_75t_SRAM      g020(.A(new_n63), .B(new_n49), .C(new_n51), .Y(new_n64));
  NAND2xp33_ASAP7_75t_SRAM     g021(.A(new_n46), .B(new_n64), .Y(N223));
  NOR2xp33_ASAP7_75t_SRAM      g022(.A(new_n45), .B(new_n64), .Y(new_n66));
  NOR3xp33_ASAP7_75t_SRAM      g023(.A(new_n66), .B(new_n44), .C(N112), .Y(new_n67));
  INVx1_ASAP7_75t_L            g024(.A(new_n67), .Y(new_n68));
  INVx1_ASAP7_75t_L            g025(.A(N99), .Y(new_n69));
  INVx1_ASAP7_75t_L            g026(.A(new_n51), .Y(new_n70));
  INVx1_ASAP7_75t_L            g027(.A(N43), .Y(new_n71));
  NOR2xp33_ASAP7_75t_SRAM      g028(.A(N37), .B(new_n71), .Y(new_n72));
  INVx1_ASAP7_75t_L            g029(.A(N4), .Y(new_n73));
  INVx1_ASAP7_75t_L            g030(.A(N17), .Y(new_n74));
  OAI22xp33_ASAP7_75t_SRAM     g031(.A1(new_n73), .A2(N1), .B1(new_n74), .B2(N11), .Y(new_n75));
  AOI211xp5_ASAP7_75t_SRAM     g032(.A1(new_n58), .A2(N30), .B(new_n75), .C(new_n72), .Y(new_n76));
  NAND4xp25_ASAP7_75t_SRAM     g033(.A(new_n76), .B(new_n70), .C(new_n53), .D(new_n55), .Y(new_n77));
  OAI21xp33_ASAP7_75t_SRAM     g034(.A1(new_n45), .A2(new_n77), .B(new_n48), .Y(new_n78));
  NAND3xp33_ASAP7_75t_SRAM     g035(.A(new_n78), .B(N95), .C(new_n69), .Y(new_n79));
  INVx1_ASAP7_75t_L            g036(.A(new_n79), .Y(new_n80));
  INVx1_ASAP7_75t_L            g037(.A(N86), .Y(new_n81));
  OAI31xp33_ASAP7_75t_SRAM     g038(.A1(new_n45), .A2(new_n63), .A3(new_n49), .B(new_n70), .Y(new_n82));
  NAND3xp33_ASAP7_75t_SRAM     g039(.A(new_n82), .B(N82), .C(new_n81), .Y(new_n83));
  NOR3xp33_ASAP7_75t_SRAM      g040(.A(new_n77), .B(new_n45), .C(new_n49), .Y(new_n84));
  NAND2xp33_ASAP7_75t_SRAM     g041(.A(new_n53), .B(new_n84), .Y(new_n85));
  INVx1_ASAP7_75t_L            g042(.A(N69), .Y(new_n86));
  NOR2xp33_ASAP7_75t_SRAM      g043(.A(N63), .B(new_n86), .Y(new_n87));
  NAND2xp33_ASAP7_75t_SRAM     g044(.A(new_n87), .B(N223), .Y(new_n88));
  NOR2xp33_ASAP7_75t_SRAM      g045(.A(N73), .B(new_n86), .Y(new_n89));
  INVx1_ASAP7_75t_L            g046(.A(new_n89), .Y(new_n90));
  AO21x1_ASAP7_75t_SRAM        g047(.A1(new_n85), .A2(new_n88), .B(new_n90), .Y(new_n91));
  INVx1_ASAP7_75t_L            g048(.A(N60), .Y(new_n92));
  INVx1_ASAP7_75t_L            g049(.A(N56), .Y(new_n93));
  NOR2xp33_ASAP7_75t_SRAM      g050(.A(N50), .B(new_n93), .Y(new_n94));
  NOR2xp33_ASAP7_75t_SRAM      g051(.A(new_n94), .B(N223), .Y(new_n95));
  AOI21xp33_ASAP7_75t_SRAM     g052(.A1(new_n46), .A2(new_n64), .B(new_n55), .Y(new_n96));
  OAI211xp5_ASAP7_75t_SRAM     g053(.A1(new_n95), .A2(new_n96), .B(N56), .C(new_n92), .Y(new_n97));
  INVx1_ASAP7_75t_L            g054(.A(N30), .Y(new_n98));
  NOR2xp33_ASAP7_75t_SRAM      g055(.A(N24), .B(new_n98), .Y(new_n99));
  NOR5xp2_ASAP7_75t_SRAM       g056(.A(new_n87), .B(new_n75), .C(new_n94), .D(new_n72), .E(new_n99), .Y(new_n100));
  NAND5xp2_ASAP7_75t_SRAM      g057(.A(new_n46), .B(new_n100), .C(new_n48), .D(new_n70), .E(new_n57), .Y(new_n101));
  OAI31xp33_ASAP7_75t_SRAM     g058(.A1(new_n45), .A2(new_n77), .A3(new_n49), .B(new_n72), .Y(new_n102));
  NOR2xp33_ASAP7_75t_SRAM      g059(.A(N47), .B(new_n71), .Y(new_n103));
  INVx1_ASAP7_75t_L            g060(.A(new_n103), .Y(new_n104));
  AOI21xp33_ASAP7_75t_SRAM     g061(.A1(new_n101), .A2(new_n102), .B(new_n104), .Y(new_n105));
  NAND5xp2_ASAP7_75t_SRAM      g062(.A(new_n46), .B(new_n100), .C(new_n48), .D(new_n70), .E(new_n59), .Y(new_n106));
  OAI31xp33_ASAP7_75t_SRAM     g063(.A1(new_n45), .A2(new_n77), .A3(new_n49), .B(new_n99), .Y(new_n107));
  NOR2xp33_ASAP7_75t_SRAM      g064(.A(N34), .B(new_n98), .Y(new_n108));
  INVx1_ASAP7_75t_L            g065(.A(new_n108), .Y(new_n109));
  AOI21xp33_ASAP7_75t_SRAM     g066(.A1(new_n106), .A2(new_n107), .B(new_n109), .Y(new_n110));
  NOR2xp33_ASAP7_75t_SRAM      g067(.A(N1), .B(new_n73), .Y(new_n111));
  INVx1_ASAP7_75t_L            g068(.A(new_n111), .Y(new_n112));
  NAND5xp2_ASAP7_75t_SRAM      g069(.A(new_n46), .B(new_n100), .C(new_n48), .D(new_n70), .E(new_n112), .Y(new_n113));
  OAI31xp33_ASAP7_75t_SRAM     g070(.A1(new_n45), .A2(new_n77), .A3(new_n49), .B(new_n111), .Y(new_n114));
  NOR2xp33_ASAP7_75t_SRAM      g071(.A(N8), .B(new_n73), .Y(new_n115));
  INVx1_ASAP7_75t_L            g072(.A(new_n115), .Y(new_n116));
  AOI21xp33_ASAP7_75t_SRAM     g073(.A1(new_n113), .A2(new_n114), .B(new_n116), .Y(new_n117));
  NOR2xp33_ASAP7_75t_SRAM      g074(.A(N11), .B(new_n74), .Y(new_n118));
  INVx1_ASAP7_75t_L            g075(.A(new_n118), .Y(new_n119));
  NAND5xp2_ASAP7_75t_SRAM      g076(.A(new_n46), .B(new_n100), .C(new_n48), .D(new_n70), .E(new_n119), .Y(new_n120));
  OAI31xp33_ASAP7_75t_SRAM     g077(.A1(new_n45), .A2(new_n77), .A3(new_n49), .B(new_n118), .Y(new_n121));
  NOR2xp33_ASAP7_75t_SRAM      g078(.A(N21), .B(new_n74), .Y(new_n122));
  INVx1_ASAP7_75t_L            g079(.A(new_n122), .Y(new_n123));
  AOI21xp33_ASAP7_75t_SRAM     g080(.A1(new_n120), .A2(new_n121), .B(new_n123), .Y(new_n124));
  NOR4xp25_ASAP7_75t_SRAM      g081(.A(new_n105), .B(new_n110), .C(new_n117), .D(new_n124), .Y(new_n125));
  NAND4xp25_ASAP7_75t_SRAM     g082(.A(new_n125), .B(new_n83), .C(new_n91), .D(new_n97), .Y(new_n126));
  NOR2xp33_ASAP7_75t_SRAM      g083(.A(new_n80), .B(new_n126), .Y(new_n127));
  NAND2xp33_ASAP7_75t_SRAM     g084(.A(new_n68), .B(new_n127), .Y(N329));
  NOR3xp33_ASAP7_75t_SRAM      g085(.A(new_n66), .B(new_n44), .C(N115), .Y(new_n129));
  OAI21xp33_ASAP7_75t_SRAM     g086(.A1(new_n67), .A2(new_n127), .B(new_n129), .Y(new_n130));
  INVx1_ASAP7_75t_L            g087(.A(N105), .Y(new_n131));
  NAND3xp33_ASAP7_75t_SRAM     g088(.A(new_n78), .B(N95), .C(new_n131), .Y(new_n132));
  O2A1O1Ixp33_ASAP7_75t_SRAM   g089(.A1(new_n67), .A2(new_n126), .B(new_n79), .C(new_n132), .Y(new_n133));
  INVx1_ASAP7_75t_L            g090(.A(new_n133), .Y(new_n134));
  INVx1_ASAP7_75t_L            g091(.A(N92), .Y(new_n135));
  NAND3xp33_ASAP7_75t_SRAM     g092(.A(new_n125), .B(new_n91), .C(new_n97), .Y(new_n136));
  OAI31xp33_ASAP7_75t_SRAM     g093(.A1(new_n67), .A2(new_n136), .A3(new_n80), .B(new_n83), .Y(new_n137));
  NAND4xp25_ASAP7_75t_SRAM     g094(.A(new_n137), .B(N82), .C(new_n135), .D(new_n82), .Y(new_n138));
  NAND3xp33_ASAP7_75t_SRAM     g095(.A(new_n127), .B(new_n68), .C(new_n91), .Y(new_n139));
  AOI21xp33_ASAP7_75t_SRAM     g096(.A1(new_n88), .A2(new_n85), .B(new_n90), .Y(new_n140));
  OAI31xp33_ASAP7_75t_SRAM     g097(.A1(new_n67), .A2(new_n126), .A3(new_n80), .B(new_n140), .Y(new_n141));
  INVx1_ASAP7_75t_L            g098(.A(N79), .Y(new_n142));
  NAND2xp33_ASAP7_75t_SRAM     g099(.A(N69), .B(new_n142), .Y(new_n143));
  AOI221xp5_ASAP7_75t_SRAM     g100(.A1(new_n85), .A2(new_n88), .B1(new_n141), .B2(new_n139), .C(new_n143), .Y(new_n144));
  NAND2xp33_ASAP7_75t_SRAM     g101(.A(new_n55), .B(new_n84), .Y(new_n145));
  INVx1_ASAP7_75t_L            g102(.A(new_n96), .Y(new_n146));
  AOI211xp5_ASAP7_75t_SRAM     g103(.A1(new_n145), .A2(new_n146), .B(new_n93), .C(N60), .Y(new_n147));
  NOR4xp25_ASAP7_75t_SRAM      g104(.A(new_n126), .B(new_n67), .C(new_n80), .D(new_n147), .Y(new_n148));
  INVx1_ASAP7_75t_L            g105(.A(new_n83), .Y(new_n149));
  INVx1_ASAP7_75t_L            g106(.A(new_n101), .Y(new_n150));
  AOI21xp33_ASAP7_75t_SRAM     g107(.A1(new_n46), .A2(new_n64), .B(new_n57), .Y(new_n151));
  OAI21xp33_ASAP7_75t_SRAM     g108(.A1(new_n151), .A2(new_n150), .B(new_n103), .Y(new_n152));
  NOR4xp25_ASAP7_75t_SRAM      g109(.A(new_n77), .B(new_n45), .C(new_n49), .D(new_n99), .Y(new_n153));
  AOI21xp33_ASAP7_75t_SRAM     g110(.A1(new_n46), .A2(new_n64), .B(new_n59), .Y(new_n154));
  OAI21xp33_ASAP7_75t_SRAM     g111(.A1(new_n154), .A2(new_n153), .B(new_n108), .Y(new_n155));
  NOR4xp25_ASAP7_75t_SRAM      g112(.A(new_n77), .B(new_n45), .C(new_n49), .D(new_n111), .Y(new_n156));
  AOI21xp33_ASAP7_75t_SRAM     g113(.A1(new_n46), .A2(new_n64), .B(new_n112), .Y(new_n157));
  OAI21xp33_ASAP7_75t_SRAM     g114(.A1(new_n157), .A2(new_n156), .B(new_n115), .Y(new_n158));
  NOR5xp2_ASAP7_75t_SRAM       g115(.A(new_n45), .B(new_n63), .C(new_n49), .D(new_n51), .E(new_n118), .Y(new_n159));
  AOI21xp33_ASAP7_75t_SRAM     g116(.A1(new_n46), .A2(new_n64), .B(new_n119), .Y(new_n160));
  OAI21xp33_ASAP7_75t_SRAM     g117(.A1(new_n159), .A2(new_n160), .B(new_n122), .Y(new_n161));
  NAND4xp25_ASAP7_75t_SRAM     g118(.A(new_n152), .B(new_n155), .C(new_n158), .D(new_n161), .Y(new_n162));
  NOR4xp25_ASAP7_75t_SRAM      g119(.A(new_n162), .B(new_n149), .C(new_n140), .D(new_n147), .Y(new_n163));
  AOI31xp33_ASAP7_75t_SRAM     g120(.A1(new_n68), .A2(new_n163), .A3(new_n79), .B(new_n97), .Y(new_n164));
  AOI211xp5_ASAP7_75t_SRAM     g121(.A1(new_n145), .A2(new_n146), .B(new_n93), .C(N66), .Y(new_n165));
  OAI21xp33_ASAP7_75t_SRAM     g122(.A1(new_n164), .A2(new_n148), .B(new_n165), .Y(new_n166));
  NOR4xp25_ASAP7_75t_SRAM      g123(.A(new_n126), .B(new_n67), .C(new_n80), .D(new_n105), .Y(new_n167));
  AOI31xp33_ASAP7_75t_SRAM     g124(.A1(new_n68), .A2(new_n163), .A3(new_n79), .B(new_n152), .Y(new_n168));
  AOI211xp5_ASAP7_75t_SRAM     g125(.A1(new_n102), .A2(new_n101), .B(new_n71), .C(N53), .Y(new_n169));
  OAI21xp33_ASAP7_75t_SRAM     g126(.A1(new_n168), .A2(new_n167), .B(new_n169), .Y(new_n170));
  NOR4xp25_ASAP7_75t_SRAM      g127(.A(new_n126), .B(new_n67), .C(new_n80), .D(new_n110), .Y(new_n171));
  AOI31xp33_ASAP7_75t_SRAM     g128(.A1(new_n68), .A2(new_n163), .A3(new_n79), .B(new_n155), .Y(new_n172));
  AOI211xp5_ASAP7_75t_SRAM     g129(.A1(new_n107), .A2(new_n106), .B(new_n98), .C(N40), .Y(new_n173));
  OAI21xp33_ASAP7_75t_SRAM     g130(.A1(new_n172), .A2(new_n171), .B(new_n173), .Y(new_n174));
  NOR4xp25_ASAP7_75t_SRAM      g131(.A(new_n126), .B(new_n67), .C(new_n80), .D(new_n117), .Y(new_n175));
  AOI31xp33_ASAP7_75t_SRAM     g132(.A1(new_n68), .A2(new_n163), .A3(new_n79), .B(new_n158), .Y(new_n176));
  AOI211xp5_ASAP7_75t_SRAM     g133(.A1(new_n114), .A2(new_n113), .B(new_n73), .C(N14), .Y(new_n177));
  OAI21xp33_ASAP7_75t_SRAM     g134(.A1(new_n176), .A2(new_n175), .B(new_n177), .Y(new_n178));
  NOR4xp25_ASAP7_75t_SRAM      g135(.A(new_n126), .B(new_n67), .C(new_n80), .D(new_n124), .Y(new_n179));
  AOI31xp33_ASAP7_75t_SRAM     g136(.A1(new_n68), .A2(new_n163), .A3(new_n79), .B(new_n161), .Y(new_n180));
  AOI211xp5_ASAP7_75t_SRAM     g137(.A1(new_n121), .A2(new_n120), .B(new_n74), .C(N27), .Y(new_n181));
  OAI21xp33_ASAP7_75t_SRAM     g138(.A1(new_n180), .A2(new_n179), .B(new_n181), .Y(new_n182));
  NAND5xp2_ASAP7_75t_SRAM      g139(.A(new_n166), .B(new_n170), .C(new_n174), .D(new_n178), .E(new_n182), .Y(new_n183));
  NOR2xp33_ASAP7_75t_SRAM      g140(.A(new_n144), .B(new_n183), .Y(new_n184));
  NAND4xp25_ASAP7_75t_SRAM     g141(.A(new_n184), .B(new_n130), .C(new_n134), .D(new_n138), .Y(N370));
  OAI21xp33_ASAP7_75t_SRAM     g142(.A1(new_n60), .A2(new_n84), .B(N4), .Y(new_n186));
  AOI221xp5_ASAP7_75t_SRAM     g143(.A1(N8), .A2(N329), .B1(N14), .B2(N370), .C(new_n186), .Y(new_n187));
  INVx1_ASAP7_75t_L            g144(.A(N115), .Y(new_n188));
  INVx1_ASAP7_75t_L            g145(.A(new_n138), .Y(new_n189));
  NOR4xp25_ASAP7_75t_SRAM      g146(.A(new_n183), .B(new_n133), .C(new_n189), .D(new_n144), .Y(new_n190));
  AND2x2_ASAP7_75t_SRAM        g147(.A(new_n190), .B(new_n130), .Y(new_n191));
  AOI22xp33_ASAP7_75t_SRAM     g148(.A1(N329), .A2(N112), .B1(N102), .B2(N223), .Y(new_n192));
  OAI211xp5_ASAP7_75t_SRAM     g149(.A1(new_n191), .A2(new_n188), .B(N108), .C(new_n192), .Y(new_n193));
  AOI22xp33_ASAP7_75t_SRAM     g150(.A1(N329), .A2(N99), .B1(N89), .B2(N223), .Y(new_n194));
  OAI211xp5_ASAP7_75t_SRAM     g151(.A1(new_n191), .A2(new_n131), .B(N95), .C(new_n194), .Y(new_n195));
  AOI22xp33_ASAP7_75t_SRAM     g152(.A1(N329), .A2(N86), .B1(N76), .B2(N223), .Y(new_n196));
  INVx1_ASAP7_75t_L            g153(.A(new_n196), .Y(new_n197));
  AOI211xp5_ASAP7_75t_SRAM     g154(.A1(N370), .A2(N92), .B(new_n50), .C(new_n197), .Y(new_n198));
  AOI22xp33_ASAP7_75t_SRAM     g155(.A1(N329), .A2(N73), .B1(N63), .B2(N223), .Y(new_n199));
  INVx1_ASAP7_75t_L            g156(.A(new_n199), .Y(new_n200));
  AOI211xp5_ASAP7_75t_SRAM     g157(.A1(N370), .A2(N79), .B(new_n86), .C(new_n200), .Y(new_n201));
  AOI22xp33_ASAP7_75t_SRAM     g158(.A1(N329), .A2(N60), .B1(N50), .B2(N223), .Y(new_n202));
  INVx1_ASAP7_75t_L            g159(.A(new_n202), .Y(new_n203));
  AOI211xp5_ASAP7_75t_SRAM     g160(.A1(N370), .A2(N66), .B(new_n93), .C(new_n203), .Y(new_n204));
  AOI22xp33_ASAP7_75t_SRAM     g161(.A1(N329), .A2(N47), .B1(N37), .B2(N223), .Y(new_n205));
  INVx1_ASAP7_75t_L            g162(.A(new_n205), .Y(new_n206));
  AOI211xp5_ASAP7_75t_SRAM     g163(.A1(N370), .A2(N53), .B(new_n71), .C(new_n206), .Y(new_n207));
  INVx1_ASAP7_75t_L            g164(.A(N27), .Y(new_n208));
  AOI22xp33_ASAP7_75t_SRAM     g165(.A1(N329), .A2(N21), .B1(N11), .B2(N223), .Y(new_n209));
  A2O1A1Ixp33_ASAP7_75t_SRAM   g166(.A1(new_n190), .A2(new_n130), .B(new_n208), .C(new_n209), .Y(new_n210));
  INVx1_ASAP7_75t_L            g167(.A(N40), .Y(new_n211));
  AOI22xp33_ASAP7_75t_SRAM     g168(.A1(N329), .A2(N34), .B1(N24), .B2(N223), .Y(new_n212));
  A2O1A1Ixp33_ASAP7_75t_SRAM   g169(.A1(new_n190), .A2(new_n130), .B(new_n211), .C(new_n212), .Y(new_n213));
  OAI22xp33_ASAP7_75t_SRAM     g170(.A1(new_n210), .A2(new_n74), .B1(new_n213), .B2(new_n98), .Y(new_n214));
  NOR5xp2_ASAP7_75t_SRAM       g171(.A(new_n198), .B(new_n214), .C(new_n201), .D(new_n204), .E(new_n207), .Y(new_n215));
  AOI31xp33_ASAP7_75t_SRAM     g172(.A1(new_n193), .A2(new_n215), .A3(new_n195), .B(new_n187), .Y(N421));
  INVx1_ASAP7_75t_L            g173(.A(N66), .Y(new_n217));
  OAI211xp5_ASAP7_75t_SRAM     g174(.A1(new_n191), .A2(new_n217), .B(N56), .C(new_n202), .Y(new_n218));
  INVx1_ASAP7_75t_L            g175(.A(new_n214), .Y(new_n219));
  OAI211xp5_ASAP7_75t_SRAM     g176(.A1(new_n191), .A2(new_n211), .B(N30), .C(new_n212), .Y(new_n220));
  NAND2xp33_ASAP7_75t_SRAM     g177(.A(new_n207), .B(new_n220), .Y(new_n221));
  NAND3xp33_ASAP7_75t_SRAM     g178(.A(new_n221), .B(new_n218), .C(new_n219), .Y(N430));
  INVx1_ASAP7_75t_L            g179(.A(N53), .Y(new_n223));
  OAI211xp5_ASAP7_75t_SRAM     g180(.A1(new_n191), .A2(new_n223), .B(N43), .C(new_n205), .Y(new_n224));
  NAND3xp33_ASAP7_75t_SRAM     g181(.A(new_n218), .B(new_n224), .C(new_n198), .Y(new_n225));
  NAND4xp25_ASAP7_75t_SRAM     g182(.A(new_n218), .B(new_n224), .C(new_n220), .D(new_n201), .Y(new_n226));
  NAND3xp33_ASAP7_75t_SRAM     g183(.A(new_n226), .B(new_n219), .C(new_n225), .Y(N431));
  NAND2xp33_ASAP7_75t_SRAM     g184(.A(new_n224), .B(new_n220), .Y(new_n228));
  NOR2xp33_ASAP7_75t_SRAM      g185(.A(new_n74), .B(new_n210), .Y(new_n229));
  AOI21xp33_ASAP7_75t_SRAM     g186(.A1(new_n207), .A2(new_n220), .B(new_n229), .Y(new_n230));
  OAI311xp33_ASAP7_75t_SRAM    g187(.A1(new_n195), .A2(new_n228), .A3(new_n198), .B1(new_n226), .C1(new_n230), .Y(N432));
endmodule


