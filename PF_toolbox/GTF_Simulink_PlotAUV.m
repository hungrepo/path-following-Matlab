function GTF_Simulink_PlotAUV(pos, orient, scale_num, time, fuselage_colour,alpha)
% =========================================================================
% Go To Formation - Plot AUV.
% =========================================================================
% Plots a 3D version of a torpedo-shaped AUV. Uses Bruce R. Land's
% Hierarchical Matlab Renderer.
% http://www.nbb.cornell.edu/neurobio/land/PROJECTS/Hierarchy/
%
% Code by: Andreas J. Haeusler
% Last Revision: October 2009
%
%
% 2009-10-07    clean new implementation
%==========================================================================

%build the fuselage
tmpsphere = UnitSphere(2);
tmpsphere.facecolor = fuselage_colour;
% nose=translate(scale(tmpsphere,3,1,1),-4.5,0,1.01);
nose=translate(scale(tmpsphere,3,1,1),-4.5,0,1.01);
tailcap=translate(scale(tmpsphere,2,1,1),4.5,0,1.01);
cyl1=UnitCylinder(4);
cyl1=translate(rotateY(scale(cyl1,1,1,5),90),0,0,1.01);
cyl1.facecolor=fuselage_colour;
fuselage = combine(cyl1, nose, tailcap);

% fins and rudders
fins=translate(scale(UnitSphere(2),.75,3,.2),5,0,1);
fins.facecolor = 'blue';
rudders=translate(rotateX(scale(UnitSphere(2),.75,3,.2),90),5,0,1);
rudders.facecolor = 'blue';
foils = combine(fins, rudders);

%motors and propellers
motor = scale(UnitSphere(2),1.5,.5,.5);
motor.facecolor = 'black';
motors = translate(motor, 5.5,0,1);
prop=rotateX(scale(UnitSphere(1.5),.1,2,.2), time*90);
prop.facecolor='black';
prop = translate(prop, 6.5, 0, 1);
motorprop = combine(motors, prop);

auv = combine(fuselage, foils, motorprop);
% the following order must be kept as it is 
% first rotate, then scale, then translate
auv = rotateX(auv, orient(1));
auv = rotateY(auv, orient(2));
auv = rotateZ(auv, orient(3) + 180);   
auv = scale(auv, scale_num, scale_num, scale_num);
auv = translate(auv, pos(1), pos(2), pos(3));

% clf
renderpatch(auv,alpha,fuselage_colour);
light('position',[0,0,10]) ;
daspect([1 1 1])

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%% The following code is copied from %%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%  Hierarchical Matlab Renderer %%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function count = renderwire(objIn,alpha)
  %hierarchical render function for structs and cell arrays
  %as a wire frame image.
  %Takes either a cell array or a single struct as input.
  %For each struct, can set:
  %  maps facecolor to edgecolor
  %  edgecolor: default='cyan'
  %  visible: default='on'

  if (iscell(objIn)) %a list of structs

     for i=1:length(objIn)

        obj=patch(objIn{i},'FaceAlpha', alpha);

        if (isfield(objIn{i},'facecolor'))
           ecolor=objIn{i}.facecolor;
        else
           ecolor=[0,1,1];
        end

        if (isfield(objIn{i},'edgecolor'))
           ecolor=objIn{i}.edgecolor;
        end

        if (isfield(objIn{i},'visible'))
           vis=objIn{i}.visible;
        else
           vis='on';
        end

        set(obj, 'FaceColor', 'none', ...
                 'EdgeColor', ecolor, ...
                 'Visible',vis);
     end  
     count=i;

   elseif (isstruct(objIn)) %must be a single struct   
      obj=patch(objIn,'FaceAlpha', alpha);

      if (isfield(objIn,'facecolor'))
           ecolor=objIn.facecolor;
        else
           ecolor=[0,1,1];
        end

        if (isfield(objIn,'edgecolor'))
           ecolor=objIn.edgecolor;
        end


        if (isfield(objIn,'visible'))
           vis=objIn.visible;
        else
           vis='on';
        end

        set(obj, 'FaceColor', 'none', ...
                 'EdgeColor', ecolor, ...
                 'Visible',vis);
        count=1;   
   end %if   
end

function count = renderpatch(objIn,alpha,fuselage_colour)
  %hierarchical render function for structs and cell arrays
  %Takes either a cell array or a single struct as input.
  %For each struct, can set:
  %  facecolor: default=cyan
  %  edgecolor: default='none'
  %  ambientstrength: default=.6
  %  specularstrength: default=.2
  %  specularexponent: default=10
  %  facelighting: default='phong'
  %  diffusestrength: default=.5
  %  visible: default='on'

  if (iscell(objIn)) %a list of structs

     for i=1:length(objIn)

        obj=patch(objIn{i},'FaceAlpha', alpha);

        if (isfield(objIn{i},'facecolor'))
           fcolor=objIn{i}.facecolor;
        else
           fcolor=[0,1,1];
        end

        if (isfield(objIn{i},'edgecolor'))
           ecolor=objIn{i}.edgecolor;
        else
%            ecolor='none';
           ecolor=fuselage_colour;
           
        end

        if (isfield(objIn{i},'ambientstrength'))
           ambstr=objIn{i}.ambientstrength;
        else
           ambstr=.6;
        end

        if (isfield(objIn{i},'specularstrength'))
           spcstr=objIn{i}.specularstrength;
        else
           spcstr=.2;
        end

        if (isfield(objIn{i},'specularexponent'))
           spcexp=objIn{i}.specularexponent;
        else
           spcexp=10;
        end

        if (isfield(objIn{i},'facelighting'))
           facelight=objIn{i}.facelighting;
        else
           facelight='phong';
        end

        if (isfield(objIn{i},'diffusestrength'))
           difstr=objIn{i}.diffusestrength;
        else
           difstr=.5;
        end

        if (isfield(objIn{i},'visible'))
           vis=objIn{i}.visible;
        else
           vis='on';
        end


        set(obj, 'FaceColor', fcolor, ...
                 'EdgeColor', ecolor, ...
                 'AmbientStrength',ambstr,...
                 'SpecularStrength',spcstr, ...
                 'SpecularExponent', spcexp, ...
                 'FaceLighting', facelight, ...
                 'DiffuseStrength', difstr, ...
                 'Visible',vis);
     end  
     count=i;

   elseif (isstruct(objIn)) %must be a single struct   
      obj=patch(objIn,'FaceAlpha', alpha);

      if (isfield(objIn,'facecolor'))
           fcolor=objIn.facecolor;
        else
           fcolor=[0,1,1];
        end

        if (isfield(objIn,'edgecolor'))
           ecolor=objIn.edgecolor;
        else
           ecolor='none';
        end

        if (isfield(objIn,'ambientstrength'))
           ambstr=objIn.ambientstrength;
        else
           ambstr=.6;
        end

        if (isfield(objIn,'specularstrength'))
           spcstr=objIn.specularstrength;
        else
           spcstr=.2;
        end

        if (isfield(objIn,'specularexponent'))
           spcexp=objIn.specularexponent;
        else
           spcexp=10;
        end

        if (isfield(objIn,'facelighting'))
           facelight=objIn.facelighting;
        else
           facelight='phong';
        end

        if (isfield(objIn,'diffusestrength'))
           difstr=objIn.diffusestrength;
        else
           difstr=.5;
        end

        if (isfield(objIn,'visible'))
           vis=objIn.visible;
        else
           vis='on';
        end

        set(obj, 'FaceColor', fcolor, ...
                 'EdgeColor', ecolor, ...
                 'AmbientStrength',ambstr,...
                 'SpecularStrength',spcstr, ...
                 'SpecularExponent', spcexp, ...
                 'FaceLighting', facelight, ...
                 'DiffuseStrength', difstr, ...
                 'Visible',vis);
        count=1;   
   end %if   
end

function objOut = combine(varargin)

  %Takes a list of opjects (structs and cell arrays) and
  %returns a cell array

  num=length(varargin);

  if (num==0)
     error('must have at least one input object');
  end

  objOut={};

  for i=1:num
     if (iscell(varargin{i})) %a list of structs
        objOut=[objOut, varargin{i}];        
     elseif (isstruct(varargin{i})) %must be a single struct         
        objOut=[objOut, {varargin{i}}];    
     else
        error('input must be s struct or cell array')
     end %if (iscell(varargin(i)))   
  end %for
end

function objOut = rotateX(objIn,a)
  %hierarchical rotate function for structs and cell arrays
  a=a/57.29;  %degrees to radians
  if (iscell(objIn)) %a list of structs
   for i=1:length(objIn)
      objOut{i}=objIn{i};
      V=objOut{i}.vertices;
      V=[V(:,1), ...
            cos(a)*V(:,2)-sin(a)*V(:,3), ...
            sin(a)*V(:,2)+cos(a)*V(:,3)];
      objOut{i}.vertices=V;   
   end      
  elseif (isstruct(objIn)) %must be a single struct   
    V=objIn.vertices;
    V=[V(:,1), ...
            cos(a)*V(:,2)-sin(a)*V(:,3), ...
            sin(a)*V(:,2)+cos(a)*V(:,3)];
    objOut=objIn;
    objOut.vertices=V; 
  else
    error('input must be s struct or cell array')
  end %if   
end

function objOut = rotateY(objIn,a)
  %hierarchical rotate function for structs and cell arrays
  a=a/57.29;  %degrees to radians
  if (iscell(objIn)) %a list of structs
   for i=1:length(objIn)
      objOut{i}=objIn{i};
      V=objOut{i}.vertices;

      V=[cos(a)*V(:,1)+sin(a)*V(:,3), ...
            V(:,2), ...
            -sin(a)*V(:,1)+cos(a)*V(:,3)];

      objOut{i}.vertices=V;   
   end      
  elseif (isstruct(objIn)) %must be a single struct   
    V=objIn.vertices;

    V=[cos(a)*V(:,1)+sin(a)*V(:,3), ...
            V(:,2), ...
            -sin(a)*V(:,1)+cos(a)*V(:,3)];

    objOut=objIn;
    objOut.vertices=V; 
  else
    error('input must be s struct or cell array')
  end %if   
end

function objOut = rotateZ(objIn,a)
  %hierarchical rotate function for structs and cell arrays
  a=a/57.29;  %degrees to radians
  if (iscell(objIn)) %a list of structs
   for i=1:length(objIn)
      objOut{i}=objIn{i};
      V=objOut{i}.vertices;

      V=[cos(a)*V(:,1)-sin(a)*V(:,2), ...
            sin(a)*V(:,1)+cos(a)*V(:,2), ...
            V(:,3)];

      objOut{i}.vertices=V;   
   end      
  elseif (isstruct(objIn)) %must be a single struct   
    V=objIn.vertices;

    V=[cos(a)*V(:,1)-sin(a)*V(:,2), ...
            sin(a)*V(:,1)+cos(a)*V(:,2), ...
            V(:,3)];

    objOut=objIn;
    objOut.vertices=V; 
  else
    error('input must be s struct or cell array')
  end %if   
end

function objOut = scale(objIn,x,y,z)
  %hierarchical scale function for structs and cell arrays

  if (iscell(objIn)) %a list of structs
   for i=1:length(objIn)
      objOut{i}=objIn{i};
      V=objIn{i}.vertices;
      V=[V(:,1)*x, V(:,2)*y, V(:,3)*z];
      objOut{i}.vertices=V;
   end      
  elseif (isstruct(objIn)) %must be a single struct   
    V=objIn.vertices;
    V=[V(:,1)*x, V(:,2)*y, V(:,3)*z];
    objOut=objIn;
    objOut.vertices=V; 
  else
    error('input must be s struct or cell array')
  end %if   
end

function objOut = translate(objIn,x,y,z)
  %hierarchical translate function for structs and cell arrays
  %Input is:
  %  an struct consisting of a vertex and face array
  %  or an cell array of structs
  %  an x,y,z translation
  if (iscell(objIn)) %a list of structs
   for i=1:length(objIn)
      objOut{i}=objIn{i};
      V=objOut{i}.vertices;
      V=[V(:,1)+x, V(:,2)+y, V(:,3)+z];
      objOut{i}.vertices=V;   
   end      
  elseif (isstruct(objIn)) %must be a single struct   
    V=objIn.vertices;
    V=[V(:,1)+x, V(:,2)+y, V(:,3)+z];
    objOut=objIn;
    objOut.vertices=V; 
  else
    error('input must be s struct or cell array')
  end %if   
end

function cube=UnitCube
  %unit cube in a format consistent with hieracrhical 
  %modeler

  %Define a cube
  cube.vertices=[ 0 0 0; 1 0 0; 1 1 0; 0 1 0; ...
        0 0 1; 1 0 1; 1 1 1; 0 1 1;] ;
  cube.faces=[ 1 2 6 5; 2 3 7 6; 3 4 8 7; 4 1 5 8; ...
        1 2 3 4; 5 6 7 8; ] ;

  cube.vertices = cube.vertices * 2 - 1;
end

function cylinder=UnitCylinder(res)
coder.extrinsic('isosurface');
  %unit sphere in a format consistent with hieracrhical 
  %modeler
  %The input paramenter is related to the sphere resolution.
  %Range 1-10. Higher number is better approximation
  %1=> 4-sided tube
  %1.5=> 8-sided tube
  %2=> 48 faces
  %3=> 80 faces
  %5=>136 faces
  %10=>272 faces

  %range check
  if (res>10)
     res=10;
  elseif (res<1)
     res=1;
  end

  res=1/res;
  [x,y,z]=meshgrid(-1-res:res:1+res, ...
     -1-res:res:1+res, -1:1:1);
  w=sqrt(x.^2+y.^2);
  cylinder=isosurface(x,y,z,w,1);

end

function sphere=UnitSphere(res)
coder.extrinsic('isosurface');
  %unit sphere in a format consistent with hieracrhical 
  %modeler
  %The input paramenter is related to the sphere resolution.
  %Range 1-10. Higher number is better approximation
  %1=>octahedron
  %1.5=> 44 faces
  %2=> 100 faces
  %2.5 => 188 faces
  %3=> 296 faces
  %5=> 900 faces
  %10=>3600 faces

  %range check
  if (res>10)
     res=10;
  elseif (res<1)
     res=1;
  end

  res=1/res;
  [x,y,z]=meshgrid(-1-res:res:1+res, ...
     -1-res:res:1+res, -1-res:res:1+res);
  w=sqrt(x.^2+y.^2+z.^2);
  sphere=isosurface(x,y,z,w,1);

end

function square=UnitSquare
  %unit square in the x-y plane
  %in a format consistent with hieracrhical 
  %modeler

   %Define a square
  square.vertices= ...
     [-1, -1, 0;
      -1, 1, 0;
       1, 1 ,0;
       1, -1, 0];
  square.faces=[1 2 3 4];
end

function surface=UnitSurface(res)
  %unit flat surface in a format consistent with hieracrhical 
  %modeler
  %The input paramenter is related to the sphere resolution.
  %Range 1-10. Higher number is better approximation
  %1=> 8 triangular faces
  %2=> 32 faces
  %5=>200 faces
  %10=>800 faces
  %20=>3200 faces
  %50=>20000 faces

  %range check
  if (res>100)
     res=100;
  elseif (res<1)
     res=1;
  end

  res=1/res;
  [x,y,z]=meshgrid(-1:res:1, ...
     -1:res:1, -1:1:1);
  w=z;
  surface=isosurface(x,y,z,w,0);

end

function torus=UnitTorus(radius, res)
  %unit torus in a format consistent with hieracrhical 
  %modeler
  %The first parameter is the radius of the cross-section
  %The second input paramenter is related to the  resolution.
  %Range 1-10. Higher number is better approximation
  %Res input of less than 3 makes a very rough torus
  %3=> 384 faces
  %5=>1230 faces
  %10=>5100 faces

  %range check
  if (res>10)
     res=10;
  elseif (res<1)
     res=1;
  end

  res=1/res;
  [x,y,z]=meshgrid(-1-radius-res:res:1+radius+res, ...
     -1-radius-res:res:1+radius+res, -radius-res:res:radius+res);

  w=(1-sqrt(x.^2+y.^2)).^2 +z.^2 - radius^2;

  torus=isosurface(x,y,z,w,0);

end