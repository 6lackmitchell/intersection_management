function [Tfxt] = adjust_fxts_time(priority)
%ADJUST_FXTS_TIME Summary of this function goes here
%   I am not acutally using this function right now, so it will not work,
%   but I want to save it for future use just in case
        if ii > 1
            [~,pIdx] = sort(priority(max(ii-1,1),1:(nAgents-nNon)),'descend');
            if ismember(aa,1:(nAgents-nNon))
                if gidx(aa) == 1
                    Tfxt(aa) = Tfxt(aa) + (pIdx(aa) - 1);
                end
            end
        end
end

