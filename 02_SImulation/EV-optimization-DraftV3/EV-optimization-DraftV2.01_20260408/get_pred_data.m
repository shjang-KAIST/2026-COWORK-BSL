function D = get_pred_data(D, S)
%GET_PRED_DATA Fill prediction-history EV fields from a source struct.

fields = get_pred_hist();
for i = 1:numel(fields)
    name = fields{i};
    D.(name) = safeget(S, name, []);
end
end

function v = safeget(S, name, default)
v = default;
if isstruct(S) && isfield(S,name) && ~isempty(S.(name))
    v = S.(name);
end
end
