clear; clc; close all;

%% Choose inputs

tirFile = "left.tir";

Fz    = 1000;       % vertical load [N]
kappa = 0.10;      % longitudinal slip ratio [-]
gamma = 0;         % camber angle [rad]
p     = 200000;    % inflation pressure [Pa]

%% Read coefficients from tire file

c = readTIRcoefficients(tirFile);

%% Constants from tire file

Fz0 = c.FNOMIN * c.LFZO;
p0  = c.NOMPRES;

%% Normalized changes

dfz = (Fz - Fz0) / Fz0;
dpi = (p - p0) / p0;

%% Pure longitudinal Magic Formula

% Shape factor
Cx = c.PCX1 * c.LCX;

% Friction coefficient
mux = (c.PDX1 + c.PDX2 * dfz) ...
    * (1 + c.PPX3 * dpi + c.PPX4 * dpi^2) ...
    * (1 - c.PDX3 * gamma^2) ...
    * c.LMUX;

% Peak factor
Dx = mux * Fz;

% Curvature factor
Ex = (c.PEX1 + c.PEX2 * dfz + c.PEX3 * dfz^2) ...
    * (1 - c.PEX4 * sign(kappa)) ...
    * c.LEX;

% Ex should usually not be above 1
Ex = min(Ex, 1);

% Longitudinal slip stiffness
Kxk = Fz ...
    * (c.PKX1 + c.PKX2 * dfz) ...
    * exp(c.PKX3 * dfz) ...
    * (1 + c.PPX1 * dpi + c.PPX2 * dpi^2) ...
    * c.LKX;

% Avoid division by zero
epsilon_x = 1e-6;

% Stiffness factor
Bx = Kxk / (Cx * Dx + epsilon_x);

% Horizontal shift
SHx = (c.PHX1 + c.PHX2 * dfz) * c.LHX;

% Shifted slip
kappax = kappa + SHx;

% Vertical shift
zeta1 = 1;
SVx = Fz * (c.PVX1 + c.PVX2 * dfz) * c.LVX * c.LMUX * zeta1;

% Final pure longitudinal force
Fx0 = Dx * sin(Cx * atan(Bx * kappax ...
    - Ex * (Bx * kappax - atan(Bx * kappax)))) + SVx;

%% Display result

fprintf("Fz      = %.4f N\n", Fz);
fprintf("kappa   = %.4f\n", kappa);
fprintf("gamma   = %.4f rad\n", gamma);
fprintf("p       = %.4f Pa\n\n", p);

fprintf("dfz     = %.6f\n", dfz);
fprintf("dpi     = %.6f\n\n", dpi);

fprintf("Cx      = %.6f\n", Cx);
fprintf("mux     = %.6f\n", mux);
fprintf("Dx      = %.6f\n", Dx);
fprintf("Ex      = %.6f\n", Ex);
fprintf("Kxk     = %.6f\n", Kxk);
fprintf("Bx      = %.6f\n", Bx);
fprintf("SHx     = %.6f\n", SHx);
fprintf("SVx     = %.6f\n\n", SVx);

fprintf("Fx0     = %.6f N\n", Fx0);

%% Optional plot of Fx0 against kappa

kappaVec = linspace(-0.3, 0.3, 300);
FxVec = zeros(size(kappaVec));

for i = 1:length(kappaVec)

    kappa_i = kappaVec(i);

    Ex_i = (c.PEX1 + c.PEX2 * dfz + c.PEX3 * dfz^2) ...
        * (1 - c.PEX4 * sign(kappa_i)) ...
        * c.LEX;

    Ex_i = min(Ex_i, 1);

    kappax_i = kappa_i + SHx;

    FxVec(i) = Dx * sin(Cx * atan(Bx * kappax_i ...
        - Ex_i * (Bx * kappax_i - atan(Bx * kappax_i)))) + SVx;
end

figure;
plot(kappaVec, FxVec, "LineWidth", 1.5);
grid on;
xlabel("\kappa");
ylabel("F_{x0} [N]");
title("Pure Longitudinal Force");

%% Local function for reading .tir coefficients

function c = readTIRcoefficients(filename)

    text = fileread(filename);

    names = [
        "FNOMIN"
        "NOMPRES"
        "LFZO"
        "LCX"
        "LMUX"
        "LEX"
        "LKX"
        "LHX"
        "LVX"
        "PCX1"
        "PDX1"
        "PDX2"
        "PDX3"
        "PEX1"
        "PEX2"
        "PEX3"
        "PEX4"
        "PKX1"
        "PKX2"
        "PKX3"
        "PHX1"
        "PHX2"
        "PVX1"
        "PVX2"
        "PPX1"
        "PPX2"
        "PPX3"
        "PPX4"
    ];

    for i = 1:length(names)

        name = names(i);

        pattern = name + "\s*=\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)";
        token = regexp(text, pattern, "tokens", "once");

        if isempty(token)
            error("Could not find coefficient: %s", name);
        end

        c.(name) = str2double(token{1});
    end
end

%% Fit longitudinal friction coefficient from kappa = 0 to kappa = 1

kappaFit = linspace(0, 1, 1000);

FxFit = zeros(size(kappaFit));
muFit = zeros(size(kappaFit));

for i = 1:length(kappaFit)

    kappa_i = kappaFit(i);

    Ex_i = (c.PEX1 + c.PEX2 * dfz + c.PEX3 * dfz^2) ...
        * (1 - c.PEX4 * sign(kappa_i)) ...
        * c.LEX;

    Ex_i = min(Ex_i, 1);

    kappax_i = kappa_i + SHx;

    FxFit(i) = Dx * sin(Cx * atan(Bx * kappax_i ...
        - Ex_i * (Bx * kappax_i - atan(Bx * kappax_i)))) + SVx;

    muFit(i) = FxFit(i) / Fz;
end

%% Print the Magic Formula equation used for the plot

fprintf("\nMagic Formula equation for this plot:\n");
fprintf("Fx0(kappa) = Dx*sin(Cx*atan(Bx*(kappa + SHx) - Ex*(Bx*(kappa + SHx) - atan(Bx*(kappa + SHx))))) + SVx\n\n");

fprintf("Numerical Magic Formula equation:\n");
fprintf("Fx0(kappa) = %.8f*sin(%.8f*atan(%.8f*(kappa + %.8f) - %.8f*(%.8f*(kappa + %.8f) - atan(%.8f*(kappa + %.8f))))) + %.8f\n\n", ...
    Dx, Cx, Bx, SHx, Ex, Bx, SHx, Bx, SHx, SVx);

fprintf("Longitudinal friction equation from Magic Formula:\n");
fprintf("mu_x(kappa) = Fx0(kappa) / Fz\n\n");

fprintf("Numerical longitudinal friction equation:\n");
fprintf("mu_x(kappa) = [%.8f*sin(%.8f*atan(%.8f*(kappa + %.8f) - %.8f*(%.8f*(kappa + %.8f) - atan(%.8f*(kappa + %.8f))))) + %.8f] / %.8f\n\n", ...
    Dx, Cx, Bx, SHx, Ex, Bx, SHx, Bx, SHx, SVx, Fz);

%% Polynomial fit for mu_x(kappa)

polyOrder = 5;

p_mu = polyfit(kappaFit, muFit, polyOrder);

muPoly = polyval(p_mu, kappaFit);

%% Print fitted polynomial equation

fprintf("Fitted polynomial equation:\n");
fprintf("mu_x(kappa) = %.8f*kappa^5 + %.8f*kappa^4 + %.8f*kappa^3 + %.8f*kappa^2 + %.8f*kappa + %.8f\n", ...
    p_mu(1), p_mu(2), p_mu(3), p_mu(4), p_mu(5), p_mu(6));

%% Plot Magic Formula and fitted polynomial

figure;
plot(kappaFit, muFit, "LineWidth", 1.5);
hold on;
plot(kappaFit, muPoly, "--", "LineWidth", 1.5);
grid on;

xlabel("\kappa");
ylabel("\mu_x = F_{x0}/F_z");
title("Longitudinal friction coefficient from \kappa = 0 to 1");
legend("Magic Formula", "Polynomial fit", "Location", "best");

%% Calculate fit error

fitError = muFit - muPoly;

RMSE = sqrt(mean(fitError.^2));
maxError = max(abs(fitError));

fprintf("\nFit quality:\n");
fprintf("RMSE      = %.8f\n", RMSE);
fprintf("Max error = %.8f\n", maxError);