function msg = RecvMsg( con, timeout )
    t = tic;
    if nargin ~= 1 
        pnet(con,'setreadtimeout',timeout );
    end
    d = pnet( con, 'read', 4, 'uint8' );
    if isempty(d)
        msg = [];
        return;
    end
    if length(d) ~= 4
        error 'RecvMsg'
    end
    msgsize = typecast(d(1:4),'uint32');
    data = pnet( con, 'read', msgsize, 'uint8' )';
    msg = hlp_deserialize(data);
%    fprintf('Receiving %d bytes took %fs\n', msgsize, toc(t) );
end
