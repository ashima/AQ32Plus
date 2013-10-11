function x = testMagCal()
  d = load('mag_ac3.dat');
 
  [DD,D1] = acc1(d); 
  [A,cp] = dToA1(DD,D1);
  [Q, L] = eig(A);
  [radii, c, Cal] = deEllipsoid(A,Q,L,cp)

  [DD,D1] = acc2(d); 
  [A,cp] = dToA2(DD,D1);
  [Q, L] = eigen3x3(A);
  [radii, c, Cal] = deEllipsoid(A,Q,L,cp)

  n = size(d)(1);

  d6 = zeros(n,1);
  for i=1:(size(d)(1)),
    d2(i,:) = (d(i,:)-c');
    d3(i,:) = d2(i,:)* Cal ;
    d4(i,:) = d2(i,:) / norm(d2(i,:));
    d5(i,:) = d3(i,:) / norm(d3(i,:));
    d6(i) = acos (d4(i,:) * d5(i,:)' ) * 180/pi;
    d7(i) = norm( d3(i,:) );
  end

  #d2(1:5,:)
  #d3(1:5,:)
  #d4(1:5,:)
  #d5(1:5,:)
  #d6(1:5)'
  #d7(1:5)

end

function [DD,D1] = acc1(d)
  ACC = zeros(1,48); DD  = zeros(9); f   = [ 1,2,2,1,2,1,2,2,2 ];

  for i=1:(size(d)(1)),
    ACC += mults( d(i,:) );
  end

  DD(tril(ones(9),0)==1) = ACC(1:45);
  DD = (DD + triu(DD',1)) .* (f'*f);

  D1 = ACC(40:48).*f;
end

function [DD,D1] = acc2(d)
  global accOmega
  accOmega = zeros(35,1);

  for i=1:(size(d)(1)),
    mcAccumulate( d(i,1), d(i,2), d(i,3) );

    [DD, D1] = mcCollectO(accOmega);
    DD = (DD + tril(DD',-1));

    DDcond(i) = (norm(DD));
  end
save("tt1.dat", 'DDcond');

end

function [A,cp] = dToA1(DD,D1)
  v = DD \ D1' ;
  A = [ v(1) v(2) v(3) ; v(2) v(4) v(5) ; v(3) v(5) v(6) ];
  cp = [ v(7); v(8); v(9) ];
end

function [A,cp] = dToA2(DD,D1)
  v = DD \ D1' ;
  [A,cp] = mcCollectA(v);
  # A = [ v(1) v(2) v(4) ; v(2) v(3) v(5) ; v(4) v(5) v(6) ];
  # cp = [ v(7); v(8); v(9) ];
end

function [radii, c, Cal] = deEllipsoid(A,Q,L,cp)
  c = - Q * inv(L) * Q' * cp;

  r = 1 / (c' * A * c + 1) ;
r
  D = sqrt( L * r );
  radii = 1 ./ diag(D) ;

  Cal = Q * D * Q';
end

function OmegaOmega = mults( x )
  O          = [ ((x' * x)(tril(ones(3),0)==1))', x ];
  OmegaOmega = [ ((O' * O)(tril(ones(9),0)==1))', x ];
end

function [Q, L] = eigen3x3(A)

  q = trace(A)/3 ;
  %p = sqrt( trace( (A - q*eye(3))**2 )/6) ;
  %B = (A - q*eye(3))/p ;
  B = A - q*eye(3) ;
  p = sqrt( trace(B*B) / 6 );
  B /= p;

  theta = acos(0.5*det(B))/3 ;
  pi23 = 2*pi/3 ;
  l0 = q + p * 2 * cos(theta + pi23 * 3 );
  l1 = q + p * 2 * cos(theta + pi23 * 1 );
  l2 = q + p * 2 * cos(theta + pi23 * 2 );

  L = zeros(3,3);
  L([1,5,9]) = [l0,l1,l2];

  %A0 = A - l0*eye(3)
  %A1 = A - l1*eye(3);
  %A2 = A - l2*eye(3);
  %v0 = cross( A0(1,:), A0(2,:) );
  %v1 = cross( A1(1,:), A1(2,:) );
  %v2 = cross( A2(1,:), A2(2,:) );
  v0 = AminusLI(A, l0)';
  v1 = AminusLI(A, l1)';
  v2 = AminusLI(A, l2)';
  
  Q = [ v0'/norm(v0) , v1'/norm(v1), v2'/norm(v2) ];
Q
end 

function v = AminusLI(A,l)
  a00 = A(1,1) - l ;
  a11 = A(2,2) - l ;

  v = [ A(2,1)*A(3,2) - A(3,1)*a11 ;
        A(3,1)*A(1,2) - A(3,2)*a00 ;
        a00*a11 - A(2,1)*A(1,2) ]  ;
end



