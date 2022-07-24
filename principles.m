
format = "heic\\rok%d-depth.png";
format2 = "heic\\rok%d.png";
bigCloud = pointCloud([0 0 0]);
cloudCollection = cell(1, 36);
for i = 1:36
    depthPath = sprintf(format, i);
    depthImage = imread(depthPath);
    depth = depthImage(:,:,1);
    
    objectPath = sprintf(format2, i);
    objectImage = imread(objectPath);
    object = objectImage(:,:,1);
    
    mask = imfill(imcomplement(imbinarize(object)), 'holes'); % black and whites, inverts, fills in holes 
    scaledmask = imresize(mask, 1/5.25);
    masked = depth.*uint8(scaledmask(:,:,1));
    
    map = double(masked);
    n = 0;
    [s1, s2] = size(map);
        
    for k = 1:s1
        for j = 1:s2
            n = n + 1;
            Points(n,1) = k;
            Points(n,2) = j;
            Points(n, 3) = map(k, j);
        end
    end
    cloud = pointCloud(Points);
    cloudCollection(i) = {cloud};
    angle = (i-1)*10;
    rotationAngle = [cosd(angle), sind(angle), 0; -sind(angle), cosd(angle), 0; 0, 0, 1];
    tform = rigid3d(rotationAngle, [0 0 0]);
    rotatedCloud = pctransform(cloud, tform);
    bigCloud = pcmerge(bigCloud, rotatedCloud, 1);
    
    % pcshow(cloud);
    % cloudcollection(i) = Points;
end
pcshow(bigCloud)

% merge point clouds w registration and stiching 
% skipping downsampling as it's already small
% https://www.mathworks.com/help/vision/ug/3-d-point-cloud-registration-and-stitching.html

fixed = cloudCollection{1}; 
moving = cloudCollection{2};

tform = pcregistericp(moving, fixed, 'Metric', 'pointToPlane', 'Extrapolate', true);
alligned = pctransform(moving, tform);

scene = pcmerge(fixed, alligned, 1);

% merging rest of clouds
accumTform = tform;
for i = 3:length(cloudCollection)
    currentCloud = cloudCollection{i};

    fixed = cloudCollection{i-1};
    moving = cloudCollection{i};

    tform = pcregistericp(moving, fixed, 'Metric', 'pointToPlane', 'Extrapolate', true);

    accumTform = affine3d(tform.T * accumTform.T);
    alligned = pctransform(currentCloud, accumTform);
    
    scene = pcmerge(scene, alligned, 1);
end

pcshow(scene)