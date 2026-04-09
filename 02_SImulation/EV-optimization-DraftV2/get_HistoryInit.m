function history = get_HistoryInit()
%GET_HISTORYINIT  
%

    history = struct();

    % window-level logs
    history.pred = struct([]);
    history.opt  = struct([]);
    history.applied_v    = [];
    history.applied_lane = [];

end
