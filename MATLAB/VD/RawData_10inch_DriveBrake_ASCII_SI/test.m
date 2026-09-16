clc; clear; 
td = tireData("B1654raw45.dat");
td.RimDiameter = 0.254;
td.OverallDiameter = 0.4064;
td.SectionWidth = 0.2032;
td.RimWidth = 0.1524; 
td.AspectRatio = 50; 
td = mean(td, "Fz"); 
td.TestMethod = "Longitudinal";
td.kappa = zeros(length(td.Fx),1);


tm = tireModel.new("MF");
tm = fit(tm,td,"Dimensions");
tm = fit(tm,td,"Fx Pure")

