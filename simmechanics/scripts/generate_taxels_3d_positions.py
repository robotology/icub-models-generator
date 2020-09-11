#!/usr/bin/env python 


# Import yarp & iDynTree python bindings 
import yarp

# See https://github.com/robotology/idyntree/pull/733#issuecomment-689508234
import pkg_resources

try:
    pkg_resources.get_distribution('iDynTree')
except pkg_resources.DistributionNotFound:
    import idyntree.bindings as iDynTree
else:
    import iDynTree

# Import argparse & numpy & scipy
import argparse
import numpy as np
import scipy.interpolate

triangles_for_patch = 16;

def get_triangle_centers_from_urdf(urdfilename, linkName, skinFrameName, trianglesNumbersList):
    iDynTree.init_numpy_helpers();
    mdlLoader= iDynTree.ModelLoader();
    mdlLoader.loadModelFromFile(urdfilename);
    kinDyn = iDynTree.KinDynComputations();
    kinDyn.loadRobotModel(mdlLoader.model());

    # The results is returned by this function already in the frame used by the skin
    centersAndNormals = {};
    centersAndNormals['centers'] = {};
    centersAndNormals['normals'] = {};

    for triangleNumber in trianglesNumbersList:
        # Get center in link frame
        triangleFrameName = linkName+"_skin_"+str(triangleNumber);
        skinFrame_H_triangleFrameName = kinDyn.getRelativeTransform(skinFrameName,triangleFrameName);
        triangleCenter_wrt_skinFrame = skinFrame_H_triangleFrameName.getPosition();
        skinFrame_R_triangleFrame = skinFrame_H_triangleFrameName.getRotation().toNumPy();
        centersAndNormals['centers'][triangleNumber] = triangleCenter_wrt_skinFrame.toNumPy();
        centersAndNormals['normals'][triangleNumber] = skinFrame_R_triangleFrame[:, 2];

    return centersAndNormals;

def truncate(f, n):
    '''Truncates/pads a float f to n decimal places without rounding'''
    s = '{}'.format(f);
    if 'e' in s or 'E' in s:
        return '{0:.{1}f}'.format(f, n);
    i, p, d = s.partition('.')
    return '.'.join([i, (d+'0'*n)[:n]]);


# export the 3d points to a skinManager "positions" compatible file
def exportSkinManagerPositionTxtFile(taxels,posx,posy,posz,normx,normy,normz,name,frame,filename,taxel_per_triangle,center_taxel):
    assert(len(taxels) == len(posx))
    assert(len(taxels) == len(posy))
    assert(len(taxels) == len(posz))
    assert(len(taxels) == len(normx))
    assert(len(taxels) == len(normy))
    assert(len(taxels) == len(normz))
    out_file = open(filename,"w");
    out_file.write("name    " + name + "\n");
    out_file.write("frame    " + frame + "\n");
    out_file.write("spatial_sampling     taxel\n");
    # the convention relative to taxel2repr is that dummy taxels are
    # mapped to -1, temperature taxel are mapped -2 and tacticle taxels
    # are mapped to the index of the taxel that is the center of the triangle
    out_file.write("taxel2Repr ( ")
    for taxelIdx in taxels:
        taxel = taxels[taxelIdx];
        if( taxel["type"] == "dummy" ):
            out_file.write(" -1 ");
        elif( taxel["type"] == "thermal"):
            out_file.write(" -2 ");
        elif( taxel["type"] == "tactile"):
            out_file.write(" " + str(taxel["triangleNumber"]*taxel_per_triangle+center_taxel) + " ");
        else:
            assert(false)

    out_file.write(" )\n");

    # out write 
    out_file.write("[calibration]\n");
    for taxelIdx in taxels:
        taxel = taxels[taxelIdx];
        if( taxel["type"] == "dummy" ):
            out_file.write("0.0000\t0.0000\t0.0000\t0.0000\t0.0000\t0.0000\n");
        elif( taxel["type"] == "thermal" or taxel["type"] == "tactile"):
            taxelIndex = taxel["index"]
            # out_file.write(str(posx[taxelIndex]) + " " +  str(posy[taxelIndex]) + " " +  str(posz[taxelIndex]) + \
            #                " " + str(normx[taxelIndex]) + " " +  str(normy[taxelIndex]) + " " + str(normz[taxelIndex]) + "\n");

            out_file.write(str(truncate(posx[taxelIndex], 4)) + "\t" + str(truncate(posy[taxelIndex], 4)) + "\t" + str(truncate(posz[taxelIndex], 4)) + \
                           "\t" + str(truncate(normx[taxelIndex], 4)) + "\t" + str(truncate(normy[taxelIndex], 4)) + "\t" + str(truncate(normz[taxelIndex], 4)) + "\n");

        else:
            assert(false)
    out_file.write("\n")


def plot_points_in_2d(valuesU, valuesV, funVal):
    import matplotlib.pyplot as plt
    import sys
    from mpl_toolkits.mplot3d import Axes3D

    # plot 3d data
    fig = plt.figure()
    ax = fig.add_axes([0, 0, 1, 1], projection='3d')

    ax.axis('equal')

    ax.plot(valuesU,valuesV,funVal,'o',c="blue");

    plt.show();

def plot_points_in_3d(valuesX, valuesY, valuesZ, unknownX, unknownY, unknownZ):
    import matplotlib.pyplot as plt
    import sys
    from mpl_toolkits.mplot3d import Axes3D

    # plot 3d data
    fig = plt.figure()
    ax = fig.add_axes([0, 0, 1, 1], projection='3d')

    ax.axis('equal')

    ax.plot(unknownX,unknownY,unknownZ,'o',c="red");
    ax.plot(valuesX,valuesY,valuesZ,'o',c="blue");

    plt.show();

def taxel_list_to_taxel_dict(taxel_list):
    taxel_dict = {};
    for i in range(0,len(taxel_list)):
        taxel_dict[i] = taxel_list[i];
    return taxel_dict;

def taxel_dict_to_taxel_list(taxel_dict):
    # If we convert a dict to a taxel_list
    # we need to have an
    maxTaxelIndex = max(taxel_dict.keys());

    assert(len(taxel_dict) == maxTaxelIndex+1);

    for i in range(0,len(taxel_dict)):
        taxel_list[i] = taxel_dict[i];

    return taxel_list;

def get_dummy_taxel():
    dummy_taxel = {}
    dummy_taxel["type"] = "dummy"
    dummy_taxel["u"] = 0.0;
    dummy_taxel["v"] = 0.0;
    dummy_taxel["x"] = 0.0;
    dummy_taxel["y"] = 0.0;
    dummy_taxel["z"] = 0.0;
    dummy_taxel["triangleNumber"] = None;
    return dummy_taxel;

class triangleCluster:
    def __init__(self):
        self.triangles = {}
        self.taxels    = {}
        self.taxel_offset = 0;

    def getTrianglesInPatch(self, patchNumber):
        assert(patchNumber >=0);
        triangleClusterOfPatch = triangleCluster();
        patchTriangleNumbers = set(get_triangle_numbers_of_given_patch(patchNumber)).intersection(set(self.triangles.keys()));
        assert( len(patchTriangleNumbers) > 0);
        triangleClusterOfPatch.triangles = { new_key: self.triangles[new_key] for new_key in patchTriangleNumbers }
        patchTaxelNumbers = get_taxel_numbers_of_given_patch(patchNumber);
        triangleClusterOfPatch.taxels = { new_key: self.taxels[new_key] for new_key in patchTaxelNumbers }
        assert(len(triangleClusterOfPatch.taxels) >= 12*len(triangleClusterOfPatch.triangles))
        triangleClusterOfPatch.taxel_offset = max(triangleClusterOfPatch.taxels.keys())+1;
        return triangleClusterOfPatch


def get_triangle_numbers_of_given_patch(patchNumber):
    return range(triangles_for_patch*patchNumber+0,triangles_for_patch*patchNumber+triangles_for_patch);

def get_taxel_numbers_of_given_patch(patchNumber):
    triangle_number_range = get_triangle_numbers_of_given_patch(patchNumber);
    return range(12*triangle_number_range[0],12*(triangle_number_range[-1]+1));

class interpolation_result(object):
  def __init__(self):
     self.unknownX = []
     self.unknownY = []
     self.unknownZ = []
     self.normX = []
     self.normY = []
     self.normZ = []

# TODO : interpolate with smoothspline to compute the normals
# def interpolate_using_smoothspline(trainingPointsU,trainingPointsV,valuesX,valuesY,valuesZ);
#       Xspline = scipy.interpolate.SmoothBivariateSpline(trainingPointsU, trainingPointsV, valuesX);
#       Yspline = scipy.interpolate.SmoothBivariateSpline(trainingPointsU, trainingPointsV, valuesY);
#       Zspline = scipy.interpolate.SmoothBivariateSpline(trainingPointsU, trainingPointsV, valuesZ);

#       unknownX = Xspline(unknownPointsU,unknownPointsV,grid=False);
#       unknownY = Yspline(unknownPointsU,unknownPointsV,grid=False);
#       unknownZ = Zspline(unknownPointsU,unknownPointsV,grid=False);


def interpolate_using_griddata(trainingPointsU,trainingPointsV,valuesX,valuesY,valuesZ,unknownPointsU,unknownPointsV,taxels,centersAndNormals,taxel_offset):
    ret = interpolation_result();

    trainingPoints = np.stack([np.array(trainingPointsU),np.array(trainingPointsV)],1);
    unknownPoints = np.stack([np.array(unknownPointsU),np.array(unknownPointsV)],1);

    ret.unknownX = scipy.interpolate.griddata(np.array(trainingPoints), np.array(valuesX), np.array(unknownPoints), method="cubic");
    ret.unknownY = scipy.interpolate.griddata(np.array(trainingPoints), np.array(valuesY), np.array(unknownPoints), method="cubic");
    ret.unknownZ = scipy.interpolate.griddata(np.array(trainingPoints), np.array(valuesZ), np.array(unknownPoints), method="cubic");
    ret.normX = np.zeros(ret.unknownX.shape);
    ret.normY = np.zeros(ret.unknownY.shape);
    ret.normZ = np.zeros(ret.unknownZ.shape);

    # the taxel outside the 2D convex hull of the triangle center, use the triangle center
    # TODO use an interpolation method
    for taxelIndex in taxels:
        taxel = taxels[taxelIndex]
        if( np.isnan(ret.unknownX[taxelIndex-taxel_offset]) and not(taxel["type"] is "dummy") ):
            ret.unknownX[taxelIndex-taxel_offset] = centersAndNormals['centers'][taxel["triangleNumber"]][0]
            ret.unknownY[taxelIndex-taxel_offset] = centersAndNormals['centers'][taxel["triangleNumber"]][1]
            ret.unknownZ[taxelIndex-taxel_offset] = centersAndNormals['centers'][taxel["triangleNumber"]][2]

        # normals TODO compute normals, for now just put the normal of the center of the triangle
        if( not(taxel["type"] is "dummy") ):
            ret.normX[taxelIndex-taxel_offset] = centersAndNormals['normals'][taxel["triangleNumber"]][0]
            ret.normY[taxelIndex-taxel_offset] = centersAndNormals['normals'][taxel["triangleNumber"]][1]
            ret.normZ[taxelIndex-taxel_offset] = centersAndNormals['normals'][taxel["triangleNumber"]][2]

    return ret;

def generate_3d_positions(args):
    # Load yarp ResourceFinder
    rf = yarp.ResourceFinder();
    rf.setVerbose();
    # Set the same default context of the iCubSkinGui for convenience loading
    # .ini position containing the 2D location of the patches
    rf.setDefaultContext("skinGui/skinGui");

    prop = yarp.Property();
    skinGuiConfFile = args.skinGui_conf_file[0];
    skinGuiConfFullName = rf.findFileByName(skinGuiConfFile);
    prop.fromConfigFile(skinGuiConfFullName);

    print("Reading 2D taxel positions from " + skinGuiConfFullName)

    sens_group = prop.findGroup("SENSORS")

    completePart = triangleCluster();

    maxTriangleNumber = -100;
    completePart.trianglesNumbersList = [];
    for i in range(1,sens_group.size()):
        triangle_group = sens_group.get(i).asList();
        triangle = {}
        triangle["type"]   = triangle_group.get(0).asString();
        triangle["number"] = triangle_group.get(1).asInt();
        maxTriangleNumber = max(triangle["number"],maxTriangleNumber);
        completePart.trianglesNumbersList.append(triangle["number"]);
        triangle["u"]      = triangle_group.get(2).asInt();
        triangle["v"]      = triangle_group.get(3).asInt();
        triangle["orient"] = triangle_group.get(4).asInt();
        triangle["gain"]   = triangle_group.get(5).asInt();
        triangle["mirror"] = triangle_group.get(6).asInt();

        completePart.triangles[triangle["number"]] = triangle

    # taxel positions in triangle frame (expressed in millimeters)
    taxelsPosInTriangle = []
    # taxel 0
    taxelsPosInTriangle.append(np.array([6.533, 0.0]))
    # taxel 1
    taxelsPosInTriangle.append(np.array([9.8, -5.66]))
    # taxel 2
    taxelsPosInTriangle.append(np.array([3.267, -5.66]))
    # taxel 3
    taxelsPosInTriangle.append(np.array([0.0, 0.0]))
    # taxel 4
    taxelsPosInTriangle.append(np.array([-3.267, -5.66]))
    # taxel 5
    taxelsPosInTriangle.append(np.array([-9.8, -5.66]))
    # taxel 6 (thermal pad!)
    taxelsPosInTriangle.append(np.array([-6.533, -3.75]))
    # taxel 7
    taxelsPosInTriangle.append(np.array([-6.533, 0]))
    # taxel 8
    taxelsPosInTriangle.append(np.array([-3.267, 5.66]))
    # taxel 9
    taxelsPosInTriangle.append(np.array([0.0, 11.317]))
    # taxel 10 (thermal pad)
    taxelsPosInTriangle.append(np.array([0, 7.507]))
    # taxel 11
    taxelsPosInTriangle.append(np.array([3.267, 5.66]))

    # generate taxel list, we allocate a list of the total number of triangles
    # dummy values (for the foot are 32) and then we overwrite the taxels
    # for the real triangles
    dummy_taxel = get_dummy_taxel();

    # the total number of the triangles is composed by both real triangles
    # and dummy triangles, is given by the length of the yarp vector published
    # on the port, divided by 12 (for the torso: 384/12 = 32).
    # Alternativly the total number of triangle for a skin part can be computed
    # from the number of patches present in the skin part: each skin patch contains
    # (a maximum of) 16 triangles, and for each skin patch 16*12 = 192 taxels are always
    # streamed in the YARP ports, regardless of the actual presense of a physical triangle
    # in the skin, so the total number of triangles is given by number_of_patches*16 .
    # For more info see http://wiki.icub.org/wiki/Tactile_sensors_(aka_Skin)
    # If we do not know apriori the number of patches in this part, we can easily get
    # the total number of triangle from the skinGui configuration file: we just need to find the triangle with
    # the maximum number, and then find the lowest multiple of 16 bigger then the maximum triangle number (plus one
    # because the triangle number is 0-based
    total_number_of_patches = (maxTriangleNumber/16)+1;
    total_number_of_triangles = total_number_of_patches*16;

    # number of taxels for triangle
    taxel_per_triangle = 12

    # list of taxels (from 0 to taxel_per_triangle) that are thermal
    thermal_taxels_list = [6,10]

    # taxel that is the center of the triangle
    center_taxel = 3;

    # pre-populate the taxels vector with all dummy taxels
    taxelsList = total_number_of_triangles*taxel_per_triangle*[dummy_taxel]

    for triangleNumber in completePart.triangles:
        triangle = completePart.triangles[triangleNumber];
        for i in range(0,taxel_per_triangle):
            theta = np.pi*triangle["orient"]/180
            rotMatrix = np.array([[np.cos(theta), -np.sin(theta)],
                                  [np.sin(theta),  np.cos(theta)]])
            offset = rotMatrix.dot(taxelsPosInTriangle[i])

            taxel = {}

            # index of the taxel in the skin part YARP port
            taxel["index"] = triangle["number"]*taxel_per_triangle+i;
            taxel["triangleNumber"] = triangle["number"]

            if( i in thermal_taxels_list ):
                taxel["type"] = "thermal"
                # u,v are the coordinates in millimeters of the taxels in
                # the iCubSkinGui
                # compute the offset of the taxel with respect to the triangle center
                taxel["u"] = triangle["u"] + offset[0]
                taxel["v"] = triangle["v"] + offset[1]

                # the taxel x, y, z position in skin frame will be filled by
                # the interpolation procedure
            else:
                taxel["type"] = "tactile"
                taxel["u"] = triangle["u"] + offset[0]
                taxel["v"] = triangle["v"] + offset[1]
                taxel["x"] = None
                taxel["y"] = None
                taxel["z"] = None
                taxelsList[taxel["index"]] = taxel

    completePart.taxels = taxel_list_to_taxel_dict(taxelsList);

    # Get triangle centers from CAD (passing through the URDF)
    centersAndNormals = get_triangle_centers_from_urdf(args.urdf[0],args.link[0],args.skin_frame[0],completePart.trianglesNumbersList);

    # Build interpolation cluster (for now depending on indipendent_patches switch)
    interpolationClusters = [];

    if( args.indipendent_patches ):
        for patchId in range(0,total_number_of_patches):
            interpolationClusters.append(completePart.getTrianglesInPatch(patchId))
    else:
        interpolationClusters.append(completePart)

    completePartUnknownX = [];
    completePartUnknownY = [];
    completePartUnknownZ = [];
    completePartCentersX = [];
    completePartCentersY = [];
    completePartCentersZ = [];
    completePartNormX = [];
    completePartNormY = [];
    completePartNormZ = [];

    for cluster in interpolationClusters:
        trainingPointsU = []
        trainingPointsV = []
        unknownPointsU  = []
        unknownPointsV  = []
        valuesX = []
        valuesY = []
        valuesZ = []
        for triangleNumber in cluster.triangles:
            trainingPointsU.append(cluster.triangles[triangleNumber]["u"]);
            trainingPointsV.append(cluster.triangles[triangleNumber]["v"]);
            valuesX.append(centersAndNormals['centers'][triangleNumber][0])
            valuesY.append(centersAndNormals['centers'][triangleNumber][1]);
            valuesZ.append(centersAndNormals['centers'][triangleNumber][2]);

        for taxelId in cluster.taxels:
            taxel = cluster.taxels[taxelId];
            unknownPointsU.append(taxel["u"]);
            unknownPointsV.append(taxel["v"]);

        assert(len(trainingPointsU) == len(trainingPointsV))
        assert(len(trainingPointsU) == len(valuesX))
        assert(len(trainingPointsU) == len(valuesY))
        assert(len(trainingPointsU) == len(valuesZ))

        ret = interpolate_using_griddata(trainingPointsU,trainingPointsV,valuesX,valuesY,valuesZ,unknownPointsU,unknownPointsV,cluster.taxels,centersAndNormals,cluster.taxel_offset);

        # Plot results
        if( args.plot ):
            plot_points_in_3d(valuesX, valuesY, valuesZ, ret.unknownX, ret.unknownY, ret.unknownZ);

        # Append the results
        completePartUnknownX.extend(ret.unknownX);
        completePartUnknownY.extend(ret.unknownY);
        completePartUnknownZ.extend(ret.unknownZ);
        completePartNormX.extend(ret.normX);
        completePartNormY.extend(ret.normY);
        completePartNormZ.extend(ret.normZ);
        completePartCentersX.extend(valuesX);
        completePartCentersY.extend(valuesY);
        completePartCentersZ.extend(valuesZ);


    # Export results
    exportSkinManagerPositionTxtFile(completePart.taxels,completePartUnknownX,completePartUnknownY,completePartUnknownZ,completePartNormX,completePartNormY,completePartNormZ,args.link[0],args.skin_frame[0],args.skinManager_conf_file[0],taxel_per_triangle,center_taxel);

def main():
    parser = argparse.ArgumentParser(description='Generate 3D positions for iCub skin taxels, from centers extracted from CAD and iCubSkinGui 2D configuration files.')
    parser.add_argument('--urdf', dest='urdf', nargs=1, action='store', required=True, help='URDF file from which the center of the triangles are extracted.')
    parser.add_argument('--link', dest='link', nargs=1, action='store', required=True, help='The taxels of the patches attached to this link will be exported')
    parser.add_argument('--skin_frame', dest='skin_frame', nargs=1, action='store', required=True, help='For compatibility with the skinDynLib library, skin quantities are not expressed in the URDF link frame, but on some other link.')
    parser.add_argument('--skinGui_conf_file', dest='skinGui_conf_file', nargs=1, action='store', required=True, help='Name of the iCubSkinGui configuration file. The file will be searched in the skinGui/skinGui YARP context.')
    parser.add_argument('--skinManager_conf_file', dest='skinManager_conf_file', nargs=1, action='store', required=True, help='Name of the generated skinManager configuration file. The file will be written in the current directory.')
    parser.add_argument('--indipendent_patches', action='store_true', help='If present, the interpolation is done indipendently for each patch.')
    parser.add_argument('--plot', action='store_true', help='If present, create a 3D plot of taxels positions.')

    args = parser.parse_args()

    generate_3d_positions(args)

# Call the main function
if __name__ == '__main__':
    main()


