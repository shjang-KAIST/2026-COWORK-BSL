function history = get_HistoryLog(history, Pred_speds, OPTpred, OPTout, param, Fixed_steps, batt_real, batt_pred_end)

if ~isfield(history,'pred') || ~isstruct(history.pred)
    history.pred = struct('pre', struct([]), 'opt', struct([]));
end
if ~isfield(history.pred,'pre') || ~isstruct(history.pred.pre)
    history.pred.pre = struct([]);
end
if ~isfield(history.pred,'opt') || ~isstruct(history.pred.opt)
    history.pred.opt = struct([]);
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
% (1) Build pre window log
% =========================================================
P = struct();
P.step0   = step0;
P.t_rel   = t_rel;
P.pos_abs = pos_abs;
P.Nx      = Nx;

P.Pred_speds = Pred_speds;
P.OPTpred = [];
if ~isempty(OPTpred)
    P.OPTpred = OPTpred;
end

P.batt_pred_end = [];
if nargin >= 8 && ~isempty(batt_pred_end)
    P.batt_pred_end = batt_pred_end;
end

P.v        = [];
P.acc      = [];
P.trq      = [];
P.rpm      = [];
P.batt_end = [];
P.v_cmd    = [];
P = get_pred_data(P, []);

if ~isempty(OPTout) && isfield(OPTout,'pred')
    P.v        = safeget(OPTout.pred,'v',[]);
    P.acc      = safeget(OPTout.pred,'acc',[]);
    P = get_pred_data(P, OPTout.pred);
    P.batt_end = safeget(OPTout.pred,'batt_end',[]);
    P.v_cmd    = safeget(OPTout.pred,'v_cmd',[]);
end

P.bound = [];
if ~isempty(OPTout) && isfield(OPTout,'bound')
    P.bound = OPTout.bound;
end

% =========================================================
% (2) Build opt window log
% =========================================================
O = struct();
O.step0   = step0;
O.t_rel   = t_rel;
O.pos_abs = pos_abs;
O.Nx      = Nx;

O.v_cmd    = [];
O.v_plot   = [];
O.acc      = [];
O.trq      = [];
O.rpm      = [];
O.batt_end = [];
O = get_pred_data(O, []);

if ~isempty(OPTout) && isfield(OPTout,'opt')
    O.v_cmd    = safeget(OPTout.opt,'v',[]);
    O.v_plot   = safeget(OPTout.opt,'v_plot',[]);
    O.acc      = safeget(OPTout.opt,'acc',[]);
    O = get_pred_data(O, OPTout.opt);
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
% (3) Append safely
% =========================================================
history.pred.pre = append_struct(history.pred.pre, P);
history.pred.opt = append_struct(history.pred.opt, O);

fprintf('[get_HistoryLog] >> Prediction data logged\n');

end

function v = safeget(S, name, default)
v = default;
if isstruct(S) && isfield(S,name) && ~isempty(S.(name))
    v = S.(name);
end
end

function A = append_struct(A, S)
if isempty(A)
    A = S;
    return;
end
if ~isstruct(A)
    A = S;
    return;
end

fa = fieldnames(A);
fs = fieldnames(S);

missingA = setdiff(fs, fa);
for i = 1:numel(missingA)
    [A.(missingA{i})] = deal([]);
end

missingS = setdiff(fa, fs);
for i = 1:numel(missingS)
    S.(missingS{i}) = [];
end

A(end+1) = orderfields(S, A(1));
end
