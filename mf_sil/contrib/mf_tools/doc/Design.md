Parking Visu Tool
=================

## Interaction between frontend and backend process ##

@startuml
actor User

box "Application Frontend Process"
control Frontend
control ClientHandler
end box

database "Shared Memory" as mem

box "Backend Process"
control ParkingVisuClient
participant BackendDllHandler
participant TrajplaVisuPluginDLL
participant "TRJPLA/TASPOSD component" as component
end box

User -> Frontend ++ : Press plan

Frontend -> ClientHandler ++ : planPath(Scene)
ClientHandler -> mem ++: Put EM into mem
return
ClientHandler -> ParkingVisuClient: send socket command\n CLIENT_PLAN_NEW_PATH
return
return

activate ParkingVisuClient
ParkingVisuClient -> ParkingVisuClient: startPlanNewPath
ParkingVisuClient -> mem ++: Fetch EM
return
ParkingVisuClient -> BackendDllHandler ++: planNewPath()
BackendDllHandler -> TrajplaVisuPluginDLL ++: geomPlanFunc()
TrajplaVisuPluginDLL -> component ++: planPath()
return path
return path
return path
ParkingVisuClient -> mem ++: Store path result
return
ParkingVisuClient -> ClientHandler ++: send SERVER_UPDATE_PATH
deactivate ParkingVisuClient
ClientHandler -> mem ++: Fetch path
return
ClientHandler -> Frontend ++: Set path
deactivate ClientHandler
Frontend -> User: Draw path
deactivate Frontend
@enduml

## Interaction between models and items ##

@startuml

actor User
participant Frontend
participant "Vehicle Model" as model
participant "Vehicle Item" as item
participant "Property View" as propview
participant "Parking Scene" as scene
participant ClientHandler

== Move vehicle by mouse ==

User -> Frontend: Move vehicle\nwith mouse
Frontend -> model: setPos(new mouse pos)

model -> propview : signal positionChanged
propview -> propview: show updated pose coordinates

model -> item: signal positionChanged
item -> item: show updated pose

model -> scene: signal positionChanged
scene -> scene: save history

== Move vehicle by coordinate change in Property View ==

User -> propview: Change pose coordinates
propview -> model: setPos(new mouse pos)

model -> item: signal positionChanged
item -> item: show updated pose

model -> scene: signal positionChanged
scene -> scene: save history

== Move vehicle from inside component ==

ClientHandler <--]: SERVER_UPDATE_EGOPOSE
ClientHandler ->  model: setPos(new mouse pos)

model -> propview : signal positionChanged
propview -> propview: show updated pose coordinates

model -> item: signal positionChanged
item -> item: show updated pose

model -> scene: signal positionChanged
scene -> scene: save history

@enduml

## Component composition ##

@startuml
title Frontend Application

note as N1 #LightCyan
  GUI class
end note


package core {
    class ParkingSceneModel {
    }

    class ParkingBoxCollectionModel {
    }

    class ParkingBoxModel {
    }

    class ParkingBoxItem #LightCyan {
    }

    class ParkingSpaceMarkingModel {
    }

    class ParkingSpaceMarkingItem #LightCyan {
    }

    class StaticObstacleModel {
    }

    class StaticObstacleItem #LightCyan {
    }

    class TargetPoseModel {
    }

    class TargetPoseItem #LightCyan {
    }

    class VehicleModel {
    }

    class VehicleItem #LightCyan {
    }

    class TrajectorySetModel {
    }

    class DebugVehicleModel {
    }

    class DebugVehicleItem #LightCyan {
    }

    class TaposdDebugModel {
    }

    class TaposdDebugItem #LightCyan {
    }

    class TargetPoseReachableAreaModel{
    }

    class TargetPoseReachableAreaItem #LightCyan {
    }

    class DebugDrawer #LightCyan {
    }

    class ParkingSceneWidget #LightCyan {
    }

    class ObjectTreeWidget #LightCyan {
    }

    class ObjectPropertiesModel {
    }

    class ObjectPropertiesItemDelegate #LightCyan {
    }
}

package app {
    class PlannerVisualization {
    }

    class ClientHandler {
    }

    class MeasurementHandler {
    }
}

class QGraphicsScene #LightCyan {
}

class QTreeView #LightCyan {
}

VehicleModel <|-- TargetPoseModel
VehicleModel <|-- DebugVehicleModel

ParkingSceneModel  *-- "0..*" StaticObstacleModel: contains >
ParkingSceneModel  *-- "8" TargetPoseModel: Target Poses
ParkingSceneModel  *-- "20" ParkingSpaceMarkingModel: Parking Space Markings
ParkingSceneModel  *-- ParkingBoxCollectionModel: Parking Boxes
ParkingSceneModel  *-- VehicleModel: Start Pose
ParkingSceneModel  *-- TaposdDebugModel
ParkingSceneModel  *-- DebugVehicleModel: Debug Vehicle
ParkingSceneModel  *-- TrajectorySetModel: Planned trajectories
ParkingSceneModel  *-- TargetPoseReachableAreaModel


ParkingBoxCollectionModel  *-- "0..*" ParkingBoxModel: contains >

StaticObstacleModel .down. StaticObstacleItem
TargetPoseModel .down. TargetPoseItem
DebugVehicleModel .down. DebugVehicleItem
TaposdDebugModel .down. TaposdDebugItem
VehicleModel .down. VehicleItem
ParkingBoxModel .down. ParkingBoxItem
ParkingSpaceMarkingModel .down. ParkingSpaceMarkingItem
TargetPoseReachableAreaModel .down. TargetPoseReachableAreaItem

PlannerVisualization *-- ParkingSceneModel
PlannerVisualization *-- ParkingSceneWidget
PlannerVisualization *-- DebugDrawer
PlannerVisualization *-- ClientHandler
PlannerVisualization *-- ObjectTreeWidget
PlannerVisualization *-- ObjectPropertiesModel
PlannerVisualization *-- MeasurementHandler
PlannerVisualization *-- QTreeView

ParkingSceneWidget *-up- QGraphicsScene
DebugDrawer .. QGraphicsScene
ParkingSceneWidget .. ParkingSceneModel
QGraphicsScene *-up- StaticObstacleItem
QGraphicsScene *-up- TargetPoseItem
QGraphicsScene *-up- DebugVehicleItem
QGraphicsScene *-up- TaposdDebugItem
QGraphicsScene *-up- VehicleItem
QGraphicsScene *-up- ParkingBoxItem
QGraphicsScene *-up- ParkingSpaceMarkingItem
QGraphicsScene *-up- TargetPoseReachableAreaItem

QTreeView *-- ObjectPropertiesItemDelegate

ClientHandler .. DebugDrawer

ObjectPropertiesModel .. ObjectPropertiesItemDelegate

@enduml

@startuml
title Backend

package visuclientinterface {
    class VisuClientInterface {
    }
}

package visuclientlib {
    class VisuClient {
    }

    class ParkingVisuClient{
    }

    class BackendDllHandler {
    }
}

entity TrajplaVisuPluginDLL

VisuClientInterface <|-- VisuClient
VisuClient <|-- ParkingVisuClient

ParkingVisuClient *-right- BackendDllHandler

BackendDllHandler *-right- TrajplaVisuPluginDLL


@enduml
