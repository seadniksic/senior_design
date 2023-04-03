from rtabmap_ros.msg import MapData
from sensor_msgs.msg import PointCloud2
from pcl_conversions import pcl_conversions

class BridgeHeader:
    def __init__(self, mapHeader):
        self.seq = mapHeader.seq
        self.stamp = mapHeader.stamp
        self.frame_id = mapHeader.frame_id
    
    def fillNewHeader(self, newHeader):
        newHeaderseq = self.seq
        newHeader.stamp = self.stamp
        newHeader.frame_id = self.frame_id

class BridgeGraph:
    def __init__(self, mapGraph):
        self.header = mapGraph.header
        self.mapToOdom = mapGraph.mapToOdom
        self.posesId = mapGraph.posesId
        self.poses = mapGraph.poses
        self.links = mapGraph.links
    
    def fillNewGraph(self, newGraph):
        newGraph.header = self.header
        newGraph.mapToOdom = self.mapToOdom
        newGraph.posesId = self.posesId
        newGraph.poses = self.poses
        newGraph.links = self.links

class BridgeNodeData:
    def __init__(self, nodeData):
        self.id = nodeData.id
        self.mapId = nodeData.mapId
        self.weight = nodeData.weight
        self.stamp = nodeData.stamp
        self.label = nodeData.label
        self.pose = nodeData.pose
        self.groundTruthPose = nodeData.groundTruthPose
        self.image = nodeData.image
        self.depth = nodeData.depth
        self.fx = nodeData.fx
        self.fy = nodeData.fy
        self.cx = nodeData.cx
        self.cy = nodeData.cy
        self.baseline = nodeData.baseline
        self.localTransform = nodeData.localTransform
        self.laserScan = nodeData.laserScan
        self.laserScanMaxPts = nodeData.laserScanMaxPts
        self.laserScanMaxRange = nodeData.laserScanMaxRange
        self.userData = nodeData.userData

        #Could also be wordIdKeys
        self.wordIds = nodeData.wordIdValues

        self.wordKpts = nodeData.wordKpts
        self.wordPts = None
    
    def fillNewNodes(self, newNodes):
        newNodes.id = self.id
        newNodes.mapId = self.mapId
        newNodes.weight = self.weight
        newNodes.stamp = self.stamp
        newNodes.label = self.label
        newNodes.pose = self.pose
        newNodes.groundTruthPose = self.groundTruthPose
        newNodes.image = self.image
        newNodes.depth = self.depth
        newNodes.fx = self.fx
        newNodes.fy = self.fy
        newNodes.cx = self.cx
        newNodes.cy = self.cy
        newNodes.baseline = self.baseline
        newNodes.localTransform = self.localTransform
        newNodes.laserScan = self.laserScan
        newNodes.laserScanMaxPts = self.laserScanMaxPts
        newNodes.laserScanMaxRange = self.laserScanMaxRange
        newNodes.userData = self.userData
        newNodes.wordIds = self.wordIds
        newNodes.wordKpts = self.wordKpts
        newNodes.wordPts = self.wordPts


class Ros1MapDataBridge:
    def __init__(self, map):
        self.header = BridgeHeader(map.header)
        self.graph = BridgeGraph(map.graph)
        self.nodes = BridgeNodeData(map.nodes)
        self.nodes.wordPts = pcl_conversions.fromPCL(map.cloud)
    
    def getMapData(self):
        newMap = MapData()

        self.header.fillNewHeader(newMap.header)
        self.graph.fillNewGraph(newMap.graph)
        self.nodes.fillNewNodes(newMap.nodes)
