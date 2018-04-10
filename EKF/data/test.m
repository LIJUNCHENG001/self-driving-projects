lasere=[];
radare=[];

for i=1:500
    if objposelaserradarsyntheticinput{i,1}=="L"
        laser=[objposelaserradarsyntheticinput{i,2} objposelaserradarsyntheticinput{i,3}];
        true=[objposelaserradarsyntheticinput{i,5} objposelaserradarsyntheticinput{i,6}];
        lasererr=laser-true;
        lasere=[lasere;lasererr];
    else
        radar=[objposelaserradarsyntheticinput{i,2} objposelaserradarsyntheticinput{i,3} objposelaserradarsyntheticinput{i,4}];
        x1=objposelaserradarsyntheticinput{i,6};
        y1=objposelaserradarsyntheticinput{i,7};
        v1=objposelaserradarsyntheticinput{i,8};
        v2=objposelaserradarsyntheticinput{i,9};
        rho=sqrt(x1^2+y1^2);
        theta=atan2(y1,x1);
        y = sum(conj([v1 v2]) .* [cos(theta) sin(theta)] );
        true=[rho theta y];
        radarerr=radar-true;
        radare=[radare;radarerr];
    end
    
end
Ql=cov(lasere);
Qr=cov(radare);

