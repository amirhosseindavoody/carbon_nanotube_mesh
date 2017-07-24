classdef cnt
    %Essentially like a struct in c. Just a convenient container for data
    properties
        num
        chirality
        length
        cylHeight
        tubeSeparation
        cylSeparation
        x
        y
        z
    end
    
    methods
        function obj = cnt(r,c) 
         if nargin == 2
            obj(r,c) = cnt;
         end
        end
    end
    
end

