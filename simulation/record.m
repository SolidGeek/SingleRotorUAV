clear 
u = udpport("datagram", "LocalPort", 8888);

index = 0;
n = 4000;

while( 1 )
    % Each time a new packet is available
    if( u.NumDatagramsAvailable )
       
        index = index + 1

        % Read one package from 
        package = read(u,1);
        data(index) = extract_tlm_package( package.Data );
    end

end