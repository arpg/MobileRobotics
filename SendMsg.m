function SendMsg( con, msg )
    data = hlp_serialize(msg);
    d = [ typecast(uint32(length(data)),'uint8')'; uint8(data) ];
%    fprintf('Sending %d byte payload\n', length(data) );
    pnet(con,'setwritetimeout', 5 );
    pnet( con, 'write', d );
end