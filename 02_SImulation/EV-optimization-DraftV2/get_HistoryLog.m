function history = get_HistoryLog(history, Pred_speds, OPTpred, OPTout, param, Fixed_steps, batt_real, batt_pred_end)

% ---- ensure history.pred/history.opt are struct arrays ----
if ~isfield(history,'pred') || isempty(history.pred) || ~isstruct(history.pred)
    history.pred = struct([]);   % reset to struct array
end
if ~isfield(history,'opt') || isempty(history.opt) || ~isstruct(history.opt)
    history.opt = struct([]);
end

Nx  = size(Pred_speds,1);
len = Nx - 1;

dt_vec = Pred_speds(1:len,5);
bad = (~isfinite(dt_vec) | dt_vec <= 0);
if any(bad), dt_vec(bad) = param.dt; end
t_rel = [0; cumsum(dt_vec(:))];

pos_abs = Pred_speds(:,4);
step0   = Fixed_steps;

% =========================================================
% (1) Build P (pred window log)
% =========================================================
P = struct();
P.step0   = step0;
P.t_rel   = t_rel;
P.pos_abs = pos_abs;
P.Nx      = Nx;

P.Pred_speds = Pred_speds;

if ~isempty(OPTpred)
    P.OPTpred = OPTpred;
else
    P.OPTpred = [];
end

if nargin >= 8 && ~isempty(batt_pred_end)
    P.batt_pred_end = batt_pred_end;
else
    P.batt_pred_end = [];
end

% attach OPTout.pred (optional)
P.v      = [];
P.acc    = [];
P.P_req  = [];
P.I_batt = [];
P.Q_Ah   = [];
P.SOC    = [];
P.Q_batt = [];
P.Rin    = [];
P.Q_loss = [];
P.trq    = [];
P.rpm    = [];
P.batt_end = [];
P.v_cmd  = [];

if ~isempty(OPTout) && isfield(OPTout,'pred')
    P.v      = safeget(OPTout.pred,'v',[]);
    P.acc    = safeget(OPTout.pred,'acc',[]);
    P.P_req  = safeget(OPTout.pred,'P_req',[]);
    P.I_batt = safeget(OPTout.pred,'I_batt',[]);
    P.Q_Ah   = safeget(OPTout.pred,'Q_Ah',[]);
    P.SOC    = safeget(OPTout.pred,'SOC',[]);
    P.Q_batt = safeget(OPTout.pred,'Q_batt',[]);
    P.Rin    = safeget(OPTout.pred,'Rin',[]);
    P.Q_loss = safeget(OPTout.pred,'Q_loss',[]);
    P.trq    = safeget(OPTout.pred,'trq',[]);
    P.rpm    = safeget(OPTout.pred,'rpm',[]);
    P.batt_end = safeget(OPTout.pred,'batt_end',[]);
    P.v_cmd  = safeget(OPTout.pred,'v_cmd',[]);
end

P.bound = [];
if ~isempty(OPTout) && isfield(OPTout,'bound')
    P.bound = OPTout.bound;
end

% =========================================================
% (2) Build O (opt window log)
% =========================================================
O = struct();
O.step0   = step0;
O.t_rel   = t_rel;
O.pos_abs = pos_abs;
O.Nx      = Nx;

O.v_cmd  = [];
O.v_plot = [];
O.acc    = [];
O.P_req  = [];
O.I_batt = [];
O.Q_Ah   = [];
O.SOC    = [];
O.Q_batt = [];
O.Rin    = [];
O.Q_loss = [];
O.trq    = [];
O.rpm    = [];
O.batt_end = [];

if ~isempty(OPTout) && isfield(OPTout,'opt')
    O.v_cmd  = safeget(OPTout.opt,'v',[]);
    O.v_plot = safeget(OPTout.opt,'v_plot',[]);
    O.acc    = safeget(OPTout.opt,'acc',[]);
    O.P_req  = safeget(OPTout.opt,'P_req',[]);
    O.I_batt = safeget(OPTout.opt,'I_batt',[]);
    O.Q_Ah   = safeget(OPTout.opt,'Q_Ah',[]);
    O.SOC    = safeget(OPTout.opt,'SOC',[]);
    O.Q_batt = safeget(OPTout.opt,'Q_batt',[]);
    O.Rin    = safeget(OPTout.opt,'Rin',[]);
    O.Q_loss = safeget(OPTout.opt,'Q_loss',[]);
    O.trq    = safeget(OPTout.opt,'trq',[]);
    O.rpm    = safeget(OPTout.opt,'rpm',[]);
    O.batt_end = safeget(OPTout.opt,'batt_end',[]);
end

O.bound = [];
if ~isempty(OPTout) && isfield(OPTout,'bound')
    O.bound = OPTout.bound;
end

O.mpc = [];
if ~isempty(OPTout) && isfield(OPTout,'mpc')
    O.mpc = OPTout.mpc;
end

O.meta = [];
if ~isempty(OPTout) && isfield(OPTout,'meta')
    O.meta = OPTout.meta;
end

O.batt0 = [];
if nargin >= 7 && ~isempty(batt_real)
    O.batt0 = batt_real;
end

% =========================================================
% (3) Append SAFELY 
% =========================================================
history.pred = append_struct(history.pred, P);
history.opt  = append_struct(history.opt,  O);

fprintf('[get_HistoryLog] >> Optimized data logged\n');

end

% ---------- helpers ----------
function v = safeget(S, name, default)
v = default;
if isstruct(S) && isfield(S,name) && ~isempty(S.(name))
    v = S.(name);
end
end

function A = append_struct(A, S)
% Ensure A is struct array with same fields as S, then append S.
if isempty(A)
    A = S;
    return;
end
if ~isstruct(A)
    A = struct([]); % force reset if corrupted
    A = S;
    return;
end

fa = fieldnames(A);
fs = fieldnames(S);

% add missing fields to A
missingA = setdiff(fs, fa);
for i = 1:numel(missingA)
    [A.(missingA{i})] = deal([]);
end

% add missing fields to S
missingS = setdiff(fa, fs);
for i = 1:numel(missingS)
    S.(missingS{i}) = [];
end

A(end+1) = orderfields(S, A(1));
end
