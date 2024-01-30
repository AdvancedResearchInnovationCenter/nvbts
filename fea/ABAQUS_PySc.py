# -*- coding: mbcs -*-
from part import *
from material import *
from section import *
from assembly import *
from step import *
from interaction import *
from load import *
from mesh import *
from optimization import *
from job import *
from sketch import *
from visualization import *
from connectorBehavior import *
import numpy as np
from odbAccess import *
from scipy.io import savemat
import math 

ttype = "3D"


def Create_Rigid_Wall():
    mdb.models['Model-1'].ConstrainedSketch(name='__profile__', sheetSize=0.1)
    mdb.models['Model-1'].sketches['__profile__'].sketchOptions.setValues(
        decimalPlaces=3)
    mdb.models['Model-1'].sketches['__profile__'].Line(point1=(-0.02, 0.0), point2=
        (0.02, 0.0))
    mdb.models['Model-1'].sketches['__profile__'].geometry.findAt((0.0, 0.0))
    mdb.models['Model-1'].sketches['__profile__'].HorizontalConstraint(
        addUndoState=False, entity=
        mdb.models['Model-1'].sketches['__profile__'].geometry.findAt((0.0, 0.0), 
        ))
    mdb.models['Model-1'].Part(dimensionality=THREE_D, name='Part-Wall', type=
        ANALYTIC_RIGID_SURFACE)
    mdb.models['Model-1'].parts['Part-Wall'].AnalyticRigidSurfExtrude(depth=0.04, 
        sketch=mdb.models['Model-1'].sketches['__profile__'])
    del mdb.models['Model-1'].sketches['__profile__']
    mdb.models['Model-1'].parts['Part-Wall'].ReferencePoint(point=(0.0, 0.0, 0.0))


def Create_sensor_shell(hh,bb):
    mdb.models['Model-1'].ConstrainedSketch(name='__profile__', sheetSize=0.1)
    mdb.models['Model-1'].sketches['__profile__'].sketchOptions.setValues(
        decimalPlaces=3)
    mdb.models['Model-1'].sketches['__profile__'].ConstructionLine(point1=(0.0, 
        -0.05), point2=(0.0, 0.05))
    mdb.models['Model-1'].sketches['__profile__'].FixedConstraint(entity=
        mdb.models['Model-1'].sketches['__profile__'].geometry[2])
    # Draw the circle
    mdb.models['Model-1'].sketches['__profile__'].ArcByCenterEnds(center=(0.0, 
        hh/1000), direction=COUNTERCLOCKWISE, point1=(0.02, 0.0), point2=(0.0, 
        0.02))
    mdb.models['Model-1'].sketches['__profile__'].CoincidentConstraint(
        addUndoState=False, entity1=
        mdb.models['Model-1'].sketches['__profile__'].vertices[2], entity2=
        mdb.models['Model-1'].sketches['__profile__'].geometry[2])
    mdb.models['Model-1'].sketches['__profile__'].CoincidentConstraint(
        addUndoState=False, entity1=
        mdb.models['Model-1'].sketches['__profile__'].vertices[1], entity2=
        mdb.models['Model-1'].sketches['__profile__'].geometry[2])
    
    mdb.models['Model-1'].Part(dimensionality=THREE_D, name='Part-Sensor', type=
        DEFORMABLE_BODY)
    mdb.models['Model-1'].parts['Part-Sensor'].BaseShellRevolve(angle=360.0, 
        flipRevolveDirection=OFF, sketch=
        mdb.models['Model-1'].sketches['__profile__'])
    del mdb.models['Model-1'].sketches['__profile__']
    mdb.models['Model-1'].parts['Part-Sensor'].ReferencePoint(point=(0.0, 0.0, 
    0.0))
    mdb.models['Model-1'].parts['Part-Sensor'].Set(edges=
        mdb.models['Model-1'].parts['Part-Sensor'].edges.getByBoundingBox(yMin=-1,yMax=0.0001), name='Set-Base')




def Create_Sensor_3D(b, T):
    mdb.models['Model-1'].ConstrainedSketch(name='__profile__', sheetSize=0.1)
    mdb.models['Model-1'].sketches['__profile__'].sketchOptions.setValues(
        decimalPlaces=3)
    mdb.models['Model-1'].sketches['__profile__'].ConstructionLine(point1=(0.0, 
        -0.05), point2=(0.0, 0.05))
    mdb.models['Model-1'].sketches['__profile__'].FixedConstraint(entity=
        mdb.models['Model-1'].sketches['__profile__'].geometry[2])
    mdb.models['Model-1'].sketches['__profile__'].ArcByCenterEnds(center=(0.0, h/1000)
        , direction=CLOCKWISE, point1=(0.0, H/1000), point2=(b/1000, 0.0))
    mdb.models['Model-1'].sketches['__profile__'].CoincidentConstraint(
        addUndoState=False, entity1=
        mdb.models['Model-1'].sketches['__profile__'].vertices[2], entity2=
        mdb.models['Model-1'].sketches['__profile__'].geometry[2])
    mdb.models['Model-1'].sketches['__profile__'].CoincidentConstraint(
        addUndoState=False, entity1=
        mdb.models['Model-1'].sketches['__profile__'].vertices[0], entity2=
        mdb.models['Model-1'].sketches['__profile__'].geometry[2])
    mdb.models['Model-1'].sketches['__profile__'].ArcByCenterEnds(center=(0.0, h/1000)
        , direction=CLOCKWISE, point1=(0.0, (H+T)/1000), point2=((((b**2+h**2)**0.5+T)**2-h**2)**0.5/1000, 0.0))
    mdb.models['Model-1'].sketches['__profile__'].CoincidentConstraint(
        addUndoState=False, entity1=
        mdb.models['Model-1'].sketches['__profile__'].vertices[3], entity2=
        mdb.models['Model-1'].sketches['__profile__'].geometry[2])
    mdb.models['Model-1'].sketches['__profile__'].Line(point1=(0.0, H/1000), 
        point2=(0.0, (H+T)/1000))
    mdb.models['Model-1'].sketches['__profile__'].VerticalConstraint(addUndoState=
        False, entity=mdb.models['Model-1'].sketches['__profile__'].geometry[5])
    mdb.models['Model-1'].sketches['__profile__'].PerpendicularConstraint(
        addUndoState=False, entity1=
        mdb.models['Model-1'].sketches['__profile__'].geometry[3], entity2=
        mdb.models['Model-1'].sketches['__profile__'].geometry[5])
    mdb.models['Model-1'].sketches['__profile__'].Line(point1=(b/1000, 0.0), 
        point2=((((b**2+h**2)**0.5+T)**2-h**2)**0.5/1000, 0.0))
    mdb.models['Model-1'].sketches['__profile__'].HorizontalConstraint(
        addUndoState=False, entity=
        mdb.models['Model-1'].sketches['__profile__'].geometry[6])
    #mdb.models['Model-1'].sketches['__profile__'].PerpendicularConstraint(
    #    addUndoState=False, entity1=
    #    mdb.models['Model-1'].sketches['__profile__'].geometry[3], entity2=
    #    mdb.models['Model-1'].sketches['__profile__'].geometry[6])
    mdb.models['Model-1'].Part(dimensionality=THREE_D, name='Part-Sensor_3D', type=
        DEFORMABLE_BODY)
    mdb.models['Model-1'].parts['Part-Sensor_3D'].BaseSolidRevolve(angle=360.0, 
        flipRevolveDirection=OFF, sketch=
        mdb.models['Model-1'].sketches['__profile__'])
    del mdb.models['Model-1'].sketches['__profile__']
    ## create 4 partitions 
    mdb.models['Model-1'].parts['Part-Sensor_3D'].DatumPlaneByPrincipalPlane(offset=0.0, 
        principalPlane=XYPLANE)
    mdb.models['Model-1'].parts['Part-Sensor_3D'].DatumPlaneByPrincipalPlane(offset=0.0, 
        principalPlane=YZPLANE)
    mdb.models['Model-1'].parts['Part-Sensor_3D'].PartitionCellByDatumPlane(cells=
        mdb.models['Model-1'].parts['Part-Sensor_3D'].cells.getByBoundingSphere((0.0,0.0,0.0), 100.0), datumPlane=
        mdb.models['Model-1'].parts['Part-Sensor_3D'].datums[2])
    mdb.models['Model-1'].parts['Part-Sensor_3D'].PartitionCellByDatumPlane(cells=
        mdb.models['Model-1'].parts['Part-Sensor_3D'].cells.getByBoundingSphere((0.0,0.0,0.0), 100.0), datumPlane=
        mdb.models['Model-1'].parts['Part-Sensor_3D'].datums[3])
    session.viewports['Viewport: 1'].partDisplay.geometryOptions.setValues(
            datumPlanes=OFF)
     
    mdb.models['Model-1'].parts['Part-Sensor_3D'].ReferencePoint(point=(0.0, 0.0, 
    0.0))
    mdb.models['Model-1'].parts['Part-Sensor_3D'].Set(faces=
        mdb.models['Model-1'].parts['Part-Sensor_3D'].faces.getByBoundingBox(yMin=-0.0001,yMax=0.0001), name='Set-Base')








            
def Create_Planes(N):
    V_Planes = [[]]
    for i in range(1,len(N)):
        try:
            p1 = mdb.models['Model-1'].parts['Part-Sensor'].DatumPlaneByPrincipalPlane(offset= 0.0, principalPlane=XYPLANE)
        except:
            p1 = mdb.models['Model-1'].parts['Part-Sensor_3D'].DatumPlaneByPrincipalPlane(offset= 0.0, principalPlane=XYPLANE)
        V_Planes[i-1].append(p1.id)
        for j in range(1,N[i]):
            try:
                p = mdb.models['Model-1'].parts['Part-Sensor'].DatumPlaneByRotation(angle=360.0*j/N[i], 
                axis=mdb.models['Model-1'].parts['Part-Sensor'].datums[1], plane=
                mdb.models['Model-1'].parts['Part-Sensor'].datums[p1.id])
                V_Planes[i-1].append( p.id)
            except:
                p = mdb.models['Model-1'].parts['Part-Sensor_3D'].DatumPlaneByRotation(angle=360.0*j/N[i], 
                axis=mdb.models['Model-1'].parts['Part-Sensor_3D'].datums[1], plane=
                mdb.models['Model-1'].parts['Part-Sensor_3D'].datums[p1.id])
                V_Planes[i-1].append( p.id)
    
        if i != len(N):
            V_Planes.append([])
    H_Planes = []
    for i in range(1,len(N)):
        zz =   (((H-h)**2-(R[i])**2)**0.5 + h )/1000
        try:
            p1 = mdb.models['Model-1'].parts['Part-Sensor'].DatumPlaneByPrincipalPlane(offset= zz, principalPlane=XZPLANE)
        except:
            p1 = mdb.models['Model-1'].parts['Part-Sensor_3D'].DatumPlaneByPrincipalPlane(offset= zz, principalPlane=XZPLANE)
        H_Planes.append(p1.id)
     
    if H > b:
        zz =   ((((H-h)**2-(R[-1])**2)**0.5 + h )/1000)/2
        try:
            p1 = mdb.models['Model-1'].parts['Part-Sensor'].DatumPlaneByPrincipalPlane(offset= zz, principalPlane=XZPLANE)
        except: 
            p1 = mdb.models['Model-1'].parts['Part-Sensor_3D'].DatumPlaneByPrincipalPlane(offset= zz, principalPlane=XZPLANE)
            
        H_Planes.append(p1.id)
     
    return V_Planes ,H_Planes
    

def partitionate_H():
    if H <= b:
        for i in range(1,len(R)):
            zz_max =   ((((H-h)**2-(R[i-1])**2)**0.5 + h ))/1000
            zz_min =   ((((H-h)**2-(R[i])**2)**0.5 + h ))/1000
            ind_plane = H_Planes[i-1]
            try:
                mdb.models['Model-1'].parts['Part-Sensor'].PartitionFaceByDatumPlane(
                    datumPlane=mdb.models['Model-1'].parts['Part-Sensor'].datums[ind_plane], faces=
                    mdb.models['Model-1'].parts['Part-Sensor'].faces.getByBoundingBox(yMin=0.0,yMax=zz_max))
            except:
                mdb.models['Model-1'].parts['Part-Sensor_3D'].PartitionFaceByDatumPlane(
                    datumPlane=mdb.models['Model-1'].parts['Part-Sensor_3D'].datums[ind_plane], faces=
                    mdb.models['Model-1'].parts['Part-Sensor_3D'].faces.getByBoundingBox(yMin=0.0,yMax=zz_max))
    else:
        for i in range(1,len(R)):
            zz_max =   ((((H-h)**2-(R[i-1])**2)**0.5 + h ))/1000
            zz_min =   ((((H-h)**2-(R[i])**2)**0.5 + h ))/1000
            ind_plane = H_Planes[i-1]
            try:
                mdb.models['Model-1'].parts['Part-Sensor'].PartitionFaceByDatumPlane(
                    datumPlane=mdb.models['Model-1'].parts['Part-Sensor'].datums[ind_plane], faces=
                    mdb.models['Model-1'].parts['Part-Sensor'].faces.getByBoundingBox(yMin=0.0,yMax=zz_max)) 
            except:           
                mdb.models['Model-1'].parts['Part-Sensor_3D'].PartitionFaceByDatumPlane(
                    datumPlane=mdb.models['Model-1'].parts['Part-Sensor_3D'].datums[ind_plane], faces=
                    mdb.models['Model-1'].parts['Part-Sensor_3D'].faces.getByBoundingBox(yMin=0.0,yMax=zz_max)) 
        
        zz_max =   ((((H-h)**2-(R[-1])**2)**0.5 + h ))/1000
        ind_plane = H_Planes[-1]
        try:
            mdb.models['Model-1'].parts['Part-Sensor'].PartitionFaceByDatumPlane(
                datumPlane=mdb.models['Model-1'].parts['Part-Sensor'].datums[ind_plane], faces=
                mdb.models['Model-1'].parts['Part-Sensor'].faces.getByBoundingBox(yMin=0.0,yMax=zz_max))   
        except:
            mdb.models['Model-1'].parts['Part-Sensor_3D'].PartitionFaceByDatumPlane(
                datumPlane=mdb.models['Model-1'].parts['Part-Sensor_3D'].datums[ind_plane], faces=
                mdb.models['Model-1'].parts['Part-Sensor_3D'].faces.getByBoundingBox(yMin=0.0,yMax=zz_max))  
            



def partitionate_V():
    for i in range(1,len(R)):
        zz_max =   ((((H-h)**2-(R[i])**2)**0.5 + h ))/1000
        try: 
            zz_min =   ((((H-h)**2-(R[i+1])**2)**0.5 + h ))/1000
        except:
            zz_min = 0.0
        if i == 1:
            zz_max =   ((((H-h)**2-(R[0])**2)**0.5 + h ))/1000
        if i >= 3:
            zz_max =   ((((H-h)**2-(R[3])**2)**0.5 + h ))/1000
        for j in range(0,N[i]/2):
            ind_plane = V_Planes[i-1][j]
            try: 
                mdb.models['Model-1'].parts['Part-Sensor'].PartitionFaceByDatumPlane(
                    datumPlane=mdb.models['Model-1'].parts['Part-Sensor'].datums[ind_plane], faces=
                    mdb.models['Model-1'].parts['Part-Sensor'].faces.getByBoundingBox(yMin=0,yMax=zz_max))
            except:
                pass
                
            try: 
                mdb.models['Model-1'].parts['Part-Sensor_3D'].PartitionFaceByDatumPlane(
                    datumPlane=mdb.models['Model-1'].parts['Part-Sensor_3D'].datums[ind_plane], faces=
                    mdb.models['Model-1'].parts['Part-Sensor_3D'].faces.getByBoundingBox(yMin=0,yMax=zz_max))
            except:
                pass



def rtpairs(r, n):        # Distribute the markers on a circle
    for i in range(len(r)):
       for j in range(n[i]):    
        yield r[i], j*(2 * np.pi / n[i])

def Create_Set_Vertice(x,y,z,model,part,set_name): # not used
    vertice = ()
    p = mdb.models[model].parts[part]
    v = p.vertices
    myVertice = v.findAt((x,y,z),)
    vertice = vertice + (v[myVertice.index:myVertice.index+1], )
    p.Set(vertices=vertice, name=set_name)



    
def Create_Material(Name, rho, E, sigma):
    mdb.models['Model-1'].Material(name=Name)
    mdb.models['Model-1'].materials[Name].Density(table=((
        rho, ), ))
    mdb.models['Model-1'].materials[Name].Hyperelastic(
        materialType=ISOTROPIC, table=((17000.0, -200.0, 23.0, 0.0, 0.0, 0.0), ), 
        testData=OFF, type=YEOH, volumetricResponse=VOLUMETRIC_DATA)

def Create_Section(Name, Thickness):
    mdb.models['Model-1'].HomogeneousShellSection(idealization=NO_IDEALIZATION, 
    integrationRule=SIMPSON, material='Ecoflex0030', name=
    Name, nodalThicknessField='', numIntPts=5, 
    poissonDefinition=DEFAULT, preIntegrate=OFF, temperature=GRADIENT, 
    thickness=Thickness, thicknessField='', thicknessModulus=None, thicknessType=
    UNIFORM, useDensity=OFF)
    
    mdb.models['Model-1'].HomogeneousSolidSection(material='Ecoflex0030', name=
    Name, thickness=None)

def Assign_Section(Name):
    try:
        mdb.models['Model-1'].parts['Part-Sensor'].SectionAssignment(offset=0.0, 
        offsetField='', offsetType=BOTTOM_SURFACE, region=Region(
        faces=mdb.models['Model-1'].parts['Part-Sensor'].faces.getByBoundingSphere( (0,0,0), 100)), sectionName=Name, 
        thicknessAssignment=FROM_SECTION)
    except:
        mdb.models['Model-1'].parts['Part-Sensor_3D'].SectionAssignment(offset=0.0, 
        offsetField='', offsetType=BOTTOM_SURFACE, region=Region(
        cells=mdb.models['Model-1'].parts['Part-Sensor_3D'].cells.getByBoundingSphere( (0,0,0), 100)), sectionName=Name, 
        thicknessAssignment=FROM_SECTION)
        
        # mdb.models['Model-1'].parts['Part-Sensor_3D'].SectionAssignment(offset=0.0, 
        # offsetField='', offsetType=MIDDLE_SURFACE, region=Region(
        # cells=mdb.models['Model-1'].parts['Part-Sensor_3D'].cells.getSequenceFromMask(mask=('[#f ]', ), )), sectionName='Section-3D-Ecoflex', 
        # thicknessAssignment=FROM_SECTION)


def Create_Assembly():
    try:
        mdb.models['Model-1'].rootAssembly.DatumCsysByDefault(CARTESIAN)
        mdb.models['Model-1'].rootAssembly.Instance(dependent=ON, name='Part-Sensor-1', 
            part=mdb.models['Model-1'].parts['Part-Sensor'])
        mdb.models['Model-1'].rootAssembly.Instance(dependent=ON, name='Part-Wall-1', 
            part=mdb.models['Model-1'].parts['Part-Wall'])
        mdb.models['Model-1'].rootAssembly.translate(instanceList=('Part-Wall-1', ), 
        vector=(0.0, H/1000, 0.0))
        mdb.models['Model-1'].rootAssembly.rotate(angle=90.0, axisDirection=(10.0, 0.0, 
        0.0), axisPoint=(0.0, 0.0, 0.0), instanceList=('Part-Sensor-1', 
        'Part-Wall-1'))
    except:
        mdb.models['Model-1'].rootAssembly.DatumCsysByDefault(CARTESIAN)
        mdb.models['Model-1'].rootAssembly.Instance(dependent=ON, name='Part-Sensor_3D-1', 
            part=mdb.models['Model-1'].parts['Part-Sensor_3D'])
        mdb.models['Model-1'].rootAssembly.Instance(dependent=ON, name='Part-Wall-1', 
            part=mdb.models['Model-1'].parts['Part-Wall'])
        mdb.models['Model-1'].rootAssembly.translate(instanceList=('Part-Wall-1', ), 
        vector=(0.0, (H+T)/1000, 0.0))
        mdb.models['Model-1'].rootAssembly.rotate(angle=90.0, axisDirection=(10.0, 0.0, 
        0.0), axisPoint=(0.0, 0.0, 0.0), instanceList=('Part-Sensor_3D-1', 
        'Part-Wall-1'))
    session.viewports['Viewport: 1'].assemblyDisplay.geometryOptions.setValues(
        datumPlanes=OFF)


def Create_Step(Name):
    mdb.models['Model-1'].rootAssembly.regenerate()
    mdb.models['Model-1'].ExplicitDynamicsStep(improvedDtMethod=ON, name=
        Name, previous='Initial')
    mdb.models['Model-1'].fieldOutputRequests['F-Output-1'].setValues(variables=(
    'S', 'E', 'U', 'RF', 'RT', 'RM', 'COORD'))   
    mdb.models['Model-1'].steps['Step-Explicit'].setValues(improvedDtMethod=ON, 
        timePeriod=!@0000Period)

def Create_Constrain_Wall(Name,Ins_Name):
    if ttype == "2D":
        mdb.models['Model-1'].RigidBody(name=Name, refPointRegion=Region(
        referencePoints=(
        mdb.models['Model-1'].rootAssembly.instances[Ins_Name].referencePoints[2], 
        )), surfaceRegion=Region(
        side2Faces=mdb.models['Model-1'].rootAssembly.instances[Ins_Name].faces.findAt(
        ((0, 0, !@0000H/1000), (0.0, 0.0,1.0)), )))
    elif ttype == "3D":
        mdb.models['Model-1'].RigidBody(name=Name, refPointRegion=Region(
        referencePoints=(
        mdb.models['Model-1'].rootAssembly.instances[Ins_Name].referencePoints[2], 
        )), surfaceRegion=Region(
        side2Faces=mdb.models['Model-1'].rootAssembly.instances[Ins_Name].faces.findAt(
        ((0, 0, (!@0000H+!@0000T)/1000), (0.0, 0.0,1.0)), )))


def Create_Contact():
    mdb.models['Model-1'].ContactProperty('IntProp-Contact')
    mdb.models['Model-1'].interactionProperties['IntProp-Contact'].TangentialBehavior(
        formulation=ROUGH)
    mdb.models['Model-1'].interactionProperties['IntProp-Contact'].NormalBehavior(
        allowSeparation=ON, constraintEnforcementMethod=DEFAULT, 
        pressureOverclosure=HARD)
    mdb.models['Model-1'].ContactExp(createStepName='Initial', name=
    'Int-Explicit-Contact')
    mdb.models['Model-1'].interactions['Int-Explicit-Contact'].includedPairs.setValuesInStep(
        stepName='Initial', useAllstar=ON)
    mdb.models['Model-1'].interactions['Int-Explicit-Contact'].contactPropertyAssignments.appendInStep(
        assignments=((GLOBAL, SELF, 'IntProp-Contact'), ), stepName='Initial')


def Boundary_Conditions():
    mdb.models['Model-1'].TabularAmplitude(data=((0.0, 0.0), (!@0000Period, 1.0)), name=
    'Amp-1', smooth=SOLVER_DEFAULT, timeSpan=STEP)
    mdb.models['Model-1'].DisplacementBC(amplitude='Amp-1', createStepName=
        'Step-Explicit', distributionType=UNIFORM, fieldName='', fixed=OFF, 
        localCsys=None, name='BC-Wall_Disp', region=Region(referencePoints=(
        mdb.models['Model-1'].rootAssembly.instances['Part-Wall-1'].referencePoints[2], 
        )), u1=0.0, u2=0.0, u3=-!@0000Depth, ur1=0.0, ur2=0.0, ur3=0.0)
    try:
        mdb.models['Model-1'].PinnedBC(createStepName='Initial', localCsys=None, name=
            'BC-3', region=
            mdb.models['Model-1'].rootAssembly.instances['Part-Sensor-1'].sets['Set-Base'])
    except:
        mdb.models['Model-1'].PinnedBC(createStepName='Initial', localCsys=None, name=
            'BC-3', region=
            mdb.models['Model-1'].rootAssembly.instances['Part-Sensor_3D-1'].sets['Set-Base'])


def Create_Mesh(X,min_len):
    if X > min_len:
        X = min_len
    try:
        mdb.models['Model-1'].parts['Part-Sensor'].setMeshControls(elemShape=QUAD,algorithm=
        MEDIAL_AXIS, regions=
        mdb.models['Model-1'].parts['Part-Sensor'].faces.getByBoundingSphere( (0,0,0), 100))
    	
        mdb.models['Model-1'].parts['Part-Sensor'].setElementType(elemTypes=(ElemType(
            elemCode=S4R, elemLibrary=EXPLICIT, secondOrderAccuracy=ON, 
            hourglassControl=DEFAULT), ElemType(elemCode=S3R, elemLibrary=EXPLICIT)), 
            regions=(mdb.models['Model-1'].parts['Part-Sensor'].faces.getByBoundingSphere( (0,0,0), 100),))
        mdb.models['Model-1'].parts['Part-Sensor'].setMeshControls(elemShape=QUAD, 
            regions=mdb.models['Model-1'].parts['Part-Sensor'].faces.getByBoundingSphere( (0,0,0), 100), technique=STRUCTURED)
        mdb.models['Model-1'].parts['Part-Sensor'].seedPart(deviationFactor=0.005, 
            minSizeFactor=0.1, size=X)
        mdb.models['Model-1'].parts['Part-Sensor'].generateMesh()
        mdb.models['Model-1'].rootAssembly.regenerate()
        mdb.models['Model-1'].parts['Part-Sensor'].Set(name='Set-All_Nodes', nodes=
            mdb.models['Model-1'].parts['Part-Sensor'].nodes.getByBoundingBox())
    except:
        mdb.models['Model-1'].parts['Part-Sensor_3D'].setElementType(elemTypes=(
            ElemType(elemCode=C3D8R, elemLibrary=EXPLICIT, secondOrderAccuracy=ON, 
            kinematicSplit=AVERAGE_STRAIN, hourglassControl=DEFAULT, 
            distortionControl=DEFAULT), ElemType(elemCode=C3D6, elemLibrary=EXPLICIT), 
            ElemType(elemCode=C3D4, elemLibrary=EXPLICIT)), regions=(
            mdb.models['Model-1'].parts['Part-Sensor_3D'].cells.getByBoundingSphere( (0,0,0), 100), ))
        
        mdb.models['Model-1'].parts['Part-Sensor_3D'].seedPart(deviationFactor=0.1, 
        minSizeFactor=0.1, size=0.001)
        
        mdb.models['Model-1'].parts['Part-Sensor_3D'].generateMesh()
        mdb.models['Model-1'].rootAssembly.regenerate()
        mdb.models['Model-1'].parts['Part-Sensor_3D'].Set(name='Set-All_Nodes', nodes=
            mdb.models['Model-1'].parts['Part-Sensor_3D'].nodes.getByBoundingBox())


def Job_Submit():
    mdb.Job(activateLoadBalancing=False, atTime=None, contactPrint=OFF, 
        description='', echoPrint=OFF, explicitPrecision=DOUBLE_PLUS_PACK, 
        historyPrint=OFF, memory=90, memoryUnits=PERCENTAGE, model='Model-1', 
        modelPrint=OFF, multiprocessingMode=DEFAULT, name='Job-1', 
        nodalOutputPrecision=SINGLE, numCpus=!@0000CPUs, numDomains=!@0000CPUs, 
        numThreadsPerMpiProcess=1, parallelizationMethodExplicit=DOMAIN, queue=None
        , resultsFormat=ODB, scratch='', type=ANALYSIS, userSubroutine='', 
        waitHours=0, waitMinutes=0)
    mdb.jobs['Job-1'].submit(consistencyChecking=OFF) 

def Create_Sets(R,N):
    try:
        mdb.models['Model-1'].parts['Part-Sensor'].Set(edges=
            mdb.models['Model-1'].parts['Part-Sensor'].edges.getByBoundingBox(yMin=-1,yMax=0.0001), name='Set-Base')
        Nodes = mdb.models['Model-1'].parts['Part-Sensor'].vertices.findAt(((0, H/1000, 0 ),),)
        for i in range(len(R)):
            r = R[i]
            for n in range(N[i]):
                t = 2 * np.pi * n / N[i]
                xx = r*np.cos(t)/1000
                yy = r*np.sin(t)/1000
                zz =  (((H-h)**2-(r)**2)**0.5 + h )/1000
                
                Nodes = Nodes + mdb.models['Model-1'].parts['Part-Sensor'].vertices.findAt(((xx, zz, yy ),),)
        mdb.models['Model-1'].parts['Part-Sensor'].Set( vertices=Nodes, name='Markers')
        mdb.models['Model-1'].parts['Part-Wall'].Set(name='Set-RP', referencePoints=(
            mdb.models['Model-1'].parts['Part-Wall'].referencePoints[2], ))
    except:
        mdb.models['Model-1'].parts['Part-Sensor_3D'].Set(edges=
            mdb.models['Model-1'].parts['Part-Sensor_3D'].edges.getByBoundingBox(yMin=-1,yMax=0.0001), name='Set-Base')
        Nodes = mdb.models['Model-1'].parts['Part-Sensor_3D'].vertices.findAt(((0, H/1000, 0 ),),)
        for i in range(len(R)):
            r = R[i]
            for n in range(N[i]):
                t = 2 * np.pi * n / N[i]
                xx = r*np.cos(t)/1000
                yy = r*np.sin(t)/1000
                zz =  (((H-h)**2-(r)**2)**0.5 + h )/1000
        
                Nodes = Nodes + mdb.models['Model-1'].parts['Part-Sensor_3D'].vertices.findAt(((xx, zz, yy ),),)
        mdb.models['Model-1'].parts['Part-Sensor_3D'].Set( vertices=Nodes, name='Markers')
        mdb.models['Model-1'].parts['Part-Wall'].Set(name='Set-RP', referencePoints=(
            mdb.models['Model-1'].parts['Part-Wall'].referencePoints[2], ))
    return Nodes

def GetMinEdgeLen():
    Edges = mdb.models['Model-1'].parts['Part-Sensor'].edges
    ML = 1000000000
    for i in range(len(Edges)):
        e = Edges[i]
        if ML > e.getSize():
            ML = e.getSize();
    return ML
    


def History_Output():
    try:
        mdb.models['Model-1'].HistoryOutputRequest(createStepName='Step-Explicit', 
            name='H-Output-1', numIntervals=20, rebar=EXCLUDE, region=
            mdb.models['Model-1'].rootAssembly.allInstances['Part-Sensor-1'].sets['Markers']
            , sectionPoints=DEFAULT, variables=('COOR1', 'COOR2', 'COOR3'))
    except:
        mdb.models['Model-1'].HistoryOutputRequest(createStepName='Step-Explicit', 
            name='H-Output-1', numIntervals=20, rebar=EXCLUDE, region=
            mdb.models['Model-1'].rootAssembly.allInstances['Part-Sensor_3D-1'].sets['Markers']
        , sectionPoints=DEFAULT, variables=('COOR1', 'COOR2', 'COOR3'))
    
    mdb.models['Model-1'].HistoryOutputRequest(createStepName='Step-Explicit',    # Ref Point Coords
        name='H-Output-2', numIntervals=20, rebar=EXCLUDE, region=
        mdb.models['Model-1'].rootAssembly.allInstances['Part-Wall-1'].sets['Set-RP']
        , sectionPoints=DEFAULT, variables=('RF1','RF2','RF3','RM1','RM2','RM3'))
#              
#    try:    
#        mdb.models['Model-1'].HistoryOutputRequest(createStepName='Step-Explicit',    # Ref Point Coords  
#            name='H-Output-3', numIntervals=20, rebar=EXCLUDE, region=
#            mdb.models['Model-1'].rootAssembly.allInstances['Part-Sensor-1'].sets['Set-Cont_Node']
#            , sectionPoints=DEFAULT, variables=('COOR1', 'COOR2', 'COOR3'))  
#    except:    
#        mdb.models['Model-1'].HistoryOutputRequest(createStepName='Step-Explicit',    # Ref Point Coords  
#            name='H-Output-3', numIntervals=20, rebar=EXCLUDE, region=
#            mdb.models['Model-1'].rootAssembly.allInstances['Part-Sensor_3D-1'].sets['Set-Cont_Node']
#        , sectionPoints=DEFAULT, variables=('COOR1', 'COOR2', 'COOR3'))  
 





def Rotate_About_X(Theta_X):
    mdb.models['Model-1'].rootAssembly.rotate(angle=Theta_X, axisDirection=(1.0, 0.0, 
    0.0), axisPoint=(0.0, 0.0, h/1000), instanceList=('Part-Wall-1', ))

def Rotate_About_Z(Theta_Z):
    mdb.models['Model-1'].rootAssembly.rotate(angle=Theta_Z, axisDirection=(0.0, 0.0, 
    1.0), axisPoint=(0.0, 0.0, h/1000), instanceList=('Part-Wall-1', ))


def contact_point():
    Theta_X = !@0000Angle_X;
    Theta_Z = !@0000Angle_Z;
    r_x =np.array([[1.0,0.0,0.0], 
        [0.0 ,cos(math.radians(Theta_X)),-sin(math.radians(Theta_X))],
        [0.0, sin(math.radians(Theta_X)), cos(math.radians(Theta_X))]]);  #rotation matrix about x
    
    r_z = np.array([  [cos(math.radians(Theta_Z)),-sin(math.radians(Theta_Z)),0.0],
        [sin(math.radians(Theta_Z)), cos(math.radians(Theta_Z)),0.0],
        [0.0, 0.0, 1.0]]);  #rotation matrix about x
    
    r = np.dot(r_z , r_x) 
    if ttype == "2D":
        v = np.dot(r ,[0.0 ,0.0 ,(!@0000H-h)/1000.0]) + np.array([0.0 ,0.0,h/1000.0])
    elif ttype== "3D":
        v = np.dot(r ,[0.0 ,0.0 ,(!@0000H+!@0000T-h)/1000.0]) + np.array([0.0 ,0.0,h/1000.0])
    
    try:
        Nodes = mdb.models['Model-1'].rootAssembly.instances['Part-Sensor-1'].nodes
    except:
        Nodes = mdb.models['Model-1'].rootAssembly.instances['Part-Sensor_3D-1'].nodes
    
    dist_0 = 10000000.0;
    for node in Nodes: 
        cords = node.coordinates
        dist = ((cords[0]-v[0])**2.0+(cords[1]-v[1])**2+(cords[2]-v[2])**2)**0.5
        if dist < dist_0:
            dist_0 = dist
            n_lab = node.label
    try:
        Node = mdb.models['Model-1'].parts['Part-Sensor'].nodes.getFromLabel(n_lab)
        mdb.models['Model-1'].parts['Part-Sensor'].Set( nodes=mdb.models['Model-1'].parts['Part-Sensor'].nodes.getByBoundingSphere((Node.coordinates),0.000001), name='Set-Cont_Node')
    except:
        Node = mdb.models['Model-1'].parts['Part-Sensor_3D'].nodes.getFromLabel(n_lab)
        mdb.models['Model-1'].parts['Part-Sensor_3D'].Set( nodes=mdb.models['Model-1'].parts['Part-Sensor_3D'].nodes.getByBoundingSphere((Node.coordinates),0.000001), name='Set-Cont_Node')
         
    return n_lab        
    
    
#########################








N =  !@0000N_list   #[1, 6, 12, 18, 24, 30, 36, 42]    # of markers
Dia =  !@0000Dia_list     #[0.0,  5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0]   # Markers' circular pattern Diamters

 

S = [ 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]#[ 1, 1, 1, 1, 1, 1, 1]       # Markers' radius (markers' size)
H = !@0000H    # Sensor Height
B = 40.0     # Sensor Base Diamter
T = !@0000T      # Sensor thickness
R = []    # Markers radii

for i in Dia:
    R.append(i/2)    # Markers radii  = Dia / 2

b = B /2       # Sensor Base radius
h=(-b**2+H**2)/(2*H)


Plane_Height = []      
for r in R[1:]:
    z =  ((H-h)**2-r**2)**0.5 + h
    Plane_Height.append((z))

markers_list = []      
for r, t in rtpairs(R, N):
    x =  r * np.cos(t)/1000
    y =  r * np.sin(t)/1000
    z =  (((H-h)**2-r**2)**0.5 + h )/1000
    s = S[R.index(r)]/10000  # sensor size 
    
    markers_list.append((x,z,y, s))




################################################









Create_Rigid_Wall()
if ttype == "2D":
    Create_sensor_shell(float(h),float(b))
elif ttype=="3D":
    Create_Sensor_3D(b, !@0000T)

V_Planes,H_Planes = Create_Planes(N)
partitionate_H()
partitionate_V()
Create_Sets(R,N)
Create_Material('Ecoflex0030', 1050.0, !@0000E, 0.42)
if ttype=="2D":
    Create_Section('Section-Shell-Ecoflex',!@0000T/1000)
    Assign_Section('Section-Shell-Ecoflex')
elif ttype=="3D":
    Create_Section('Section-3D-Ecoflex',!@0000T/1000)
    Assign_Section('Section-3D-Ecoflex')

Create_Assembly()
Create_Step('Step-Explicit')
Create_Constrain_Wall('Constraint-Wall','Part-Wall-1')
Create_Contact()
Boundary_Conditions()
ML = !@0000ML
Create_Mesh(ML, ML)
n_lab = contact_point()
History_Output()
Rotate_About_X(!@0000Angle_X)
Rotate_About_Z(!@0000Angle_Z)
mdb.saveAs( pathName=r'!@0000CAE')  # Save the model 
Job_Submit()
 









f = open("NextCaseTrigger-CaseRunning.txt", "w")
f.write("This file to trigger the following simulation")
f.close()


mdb.jobs['Job-1'].waitForCompletion() 

f = open("NextCaseTrigger-CaseDone.txt", "w")
f.write("This file to trigger the following simulation")
f.close()
 


from odbAccess import *
from scipy.io import savemat
db =  session.openOdb(name = 'Job-1.odb')
step1 = db.steps.values()[0]

markers = db.steps[step1.name].historyRegions
Data_Arr = {"marker":{"x":[],"y":[],"z":[],"Rx":[],"Ry":[],"Rz":[]}}

l_end = len(markers.keys())
i = 1
for key in markers.keys():
    if i != l_end:
        coordX = db.steps[step1.name].historyRegions[key].historyOutputs['COOR1'].data
        coordY = db.steps[step1.name].historyRegions[key].historyOutputs['COOR2'].data
        coordZ = db.steps[step1.name].historyRegions[key].historyOutputs['COOR3'].data
        Data_Arr["marker"]["x"].append(coordX)
        Data_Arr["marker"]["y"].append(coordY)
        Data_Arr["marker"]["z"].append(coordZ)
    elif i == l_end:
        Rx = db.steps[step1.name].historyRegions[key].historyOutputs['RF1'].data
        Ry = db.steps[step1.name].historyRegions[key].historyOutputs['RF2'].data
        Rz = db.steps[step1.name].historyRegions[key].historyOutputs['RF3'].data
        Data_Arr["marker"]["Rx"].append(Rx)
        Data_Arr["marker"]["Ry"].append(Ry)
        Data_Arr["marker"]["Rz"].append(Rz)
    i+=1

savemat("Data_Arr.mat", Data_Arr)


mdb.saveAs( pathName='!@0000CAE')   # Save the model 