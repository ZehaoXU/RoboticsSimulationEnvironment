
function [p_g, PG_r, PG_y] = invScanPoint(r, y)
    
    if nargout == 1
        p_r = getInvMeasurement(y);     % Convert range-bearing measurement to local point
        p_g = transToGlobal(r, p_r);
    else
        % Compute Jacobians:
        [p_r, PR_y] = getInvMeasurement(y);
        [p_g, PG_r, PG_pr] = transToGlobal(r, p_r);
        
        % From the chain rule, we deduce:
        PG_y = PG_pr * PR_y;
    end
end