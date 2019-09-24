/****************************************************************************
 *
 *   (c) 2009-2016 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>
 *
 * QGroundControl is licensed according to the terms in the file
 * COPYING.md in the root of the source code directory.
 *
 ****************************************************************************/

#pragma once

#include <QObject>
#include <QVariantList>
#include <QGeoCoordinate>

#include "FactGroup.h"
#include "LinkInterface.h"
#include "QGCMAVLink.h"
#include "QmlObjectListModel.h"
#include "MAVLinkProtocol.h"
#include "UASMessageHandler.h"
#include "SettingsFact.h"

class UAS;
class UASInterface;
class FirmwarePlugin;
class FirmwarePluginManager;
class AutoPilotPlugin;
class MissionManager;
class GeoFenceManager;
class RallyPointManager;
class ParameterManager;
class JoystickManager;
class UASMessage;
class SettingsManager;
class ADSBVehicle;
class QGCCameraManager;

Q_DECLARE_LOGGING_CATEGORY(VehicleLog)

class Vehicle;

class VehicleDistanceSensorFactGroup : public FactGroup
{
    Q_OBJECT

public:
    VehicleDistanceSensorFactGroup(QObject* parent = NULL);

    Q_PROPERTY(Fact* rotationNone       READ rotationNone       CONSTANT)
    Q_PROPERTY(Fact* rotationYaw45      READ rotationYaw45      CONSTANT)
    Q_PROPERTY(Fact* rotationYaw90      READ rotationYaw90      CONSTANT)
    Q_PROPERTY(Fact* rotationYaw135     READ rotationYaw135     CONSTANT)
    Q_PROPERTY(Fact* rotationYaw180     READ rotationYaw180     CONSTANT)
    Q_PROPERTY(Fact* rotationYaw225     READ rotationYaw225     CONSTANT)
    Q_PROPERTY(Fact* rotationYaw270     READ rotationYaw270     CONSTANT)
    Q_PROPERTY(Fact* rotationYaw315     READ rotationYaw315     CONSTANT)
    Q_PROPERTY(Fact* rotationPitch90    READ rotationPitch90    CONSTANT)
    Q_PROPERTY(Fact* rotationPitch270   READ rotationPitch270   CONSTANT)

    Fact* rotationNone      (void) { return &_rotationNoneFact; }
    Fact* rotationYaw45     (void) { return &_rotationYaw45Fact; }
    Fact* rotationYaw90     (void) { return &_rotationYaw90Fact; }
    Fact* rotationYaw135    (void) { return &_rotationYaw90Fact; }
    Fact* rotationYaw180    (void) { return &_rotationYaw180Fact; }
    Fact* rotationYaw225    (void) { return &_rotationYaw180Fact; }
    Fact* rotationYaw270    (void) { return &_rotationYaw270Fact; }
    Fact* rotationYaw315    (void) { return &_rotationYaw315Fact; }
    Fact* rotationPitch90   (void) { return &_rotationPitch90Fact; }
    Fact* rotationPitch270  (void) { return &_rotationPitch270Fact; }

    bool idSet(void) { return _idSet; }
    void setIdSet(bool idSet) { _idSet = idSet; }
    uint8_t id(void) { return _id; }
    void setId(uint8_t id) { _id = id; }

    static const char* _rotationNoneFactName;
    static const char* _rotationYaw45FactName;
    static const char* _rotationYaw90FactName;
    static const char* _rotationYaw135FactName;
    static const char* _rotationYaw180FactName;
    static const char* _rotationYaw225FactName;
    static const char* _rotationYaw270FactName;
    static const char* _rotationYaw315FactName;
    static const char* _rotationPitch90FactName;
    static const char* _rotationPitch270FactName;

private:
    Fact _rotationNoneFact;
    Fact _rotationYaw45Fact;
    Fact _rotationYaw90Fact;
    Fact _rotationYaw135Fact;
    Fact _rotationYaw180Fact;
    Fact _rotationYaw225Fact;
    Fact _rotationYaw270Fact;
    Fact _rotationYaw315Fact;
    Fact _rotationPitch90Fact;
    Fact _rotationPitch270Fact;

    bool    _idSet; // true: _id is set to seen sensor id
    uint8_t _id;    // The id for the sensor being tracked. Current support for only a single sensor.
};


class VehicleMeteoFactGroup : public FactGroup
{
    Q_OBJECT

public:
    VehicleMeteoFactGroup(QObject* parent = NULL);
    Q_PROPERTY(Fact* temperature             READ temperature             CONSTANT)
    Q_PROPERTY(Fact* humidity                READ humidity                CONSTANT)
    Q_PROPERTY(Fact* pt100                   READ pt100                   CONSTANT)
    Q_PROPERTY(Fact* t_pot_v                 READ t_pot_v                 CONSTANT)
    Q_PROPERTY(Fact* q_hu                    READ q_hu                    CONSTANT)
    Fact* temperature         (void) { return &_temperatureFact; }
    Fact* humidity            (void) { return &_humidityFact; }
    Fact* t_pot_v             (void) { return &_t_pot_vFact; }
    Fact* q_hu                (void) { return &_q_huFact; }
    Fact* pt100               (void) { return &_pt100Fact; }

    static const char* _temperatureFactName;
    static const char* _humidityFactName;
    static const char* _t_pot_vFactName;
    static const char* _q_huFactName;
    static const char* _pt100FactName;

private:
    Fact        _temperatureFact;
    Fact        _humidityFact;
    Fact        _t_pot_vFact;
    Fact        _q_huFact;
    Fact        _pt100Fact;
};

class VehicleInsFactGroup : public FactGroup
{
    Q_OBJECT

public:
    VehicleInsFactGroup(QObject* parent = NULL);
    Q_PROPERTY(Fact* roll                 READ roll                 CONSTANT)
    Q_PROPERTY(Fact* pitch                READ pitch                CONSTANT)
    Q_PROPERTY(Fact* yaw                  READ yaw                  CONSTANT)
    Q_PROPERTY(Fact* ax                   READ ax                   CONSTANT)
    Q_PROPERTY(Fact* ay                   READ ay                   CONSTANT)
    Q_PROPERTY(Fact* az                   READ az                   CONSTANT)
    Q_PROPERTY(Fact* gx                   READ gx                   CONSTANT)
    Q_PROPERTY(Fact* gy                   READ gy                   CONSTANT)
    Q_PROPERTY(Fact* gz                   READ gz                   CONSTANT)
    Q_PROPERTY(Fact* mx                   READ mx                   CONSTANT)
    Q_PROPERTY(Fact* my                   READ my                   CONSTANT)
    Q_PROPERTY(Fact* mz                   READ mz                   CONSTANT)
    Q_PROPERTY(Fact* vns                  READ vns                  CONSTANT)
    Q_PROPERTY(Fact* vew                  READ vew                  CONSTANT)
    Q_PROPERTY(Fact* vud                  READ vud                  CONSTANT)
    Fact* roll         (void) { return &_rollFact; }
    Fact* pitch            (void) { return &_pitchFact; }
    Fact* yaw            (void) { return &_yawFact; }
    Fact* ax             (void) { return &_axFact; }
    Fact* ay             (void) { return &_ayFact; }
    Fact* az             (void) { return &_azFact; }
    Fact* gx             (void) { return &_gxFact; }
    Fact* gy             (void) { return &_gyFact; }
    Fact* gz             (void) { return &_gzFact; }
    Fact* mx             (void) { return &_mxFact; }
    Fact* my             (void) { return &_myFact; }
    Fact* mz             (void) { return &_mzFact; }
    Fact* vns            (void) { return &_vnsFact; }
    Fact* vew            (void) { return &_vewFact; }
    Fact* vud            (void) { return &_vudFact; }

    static const char* _rollFactName;
    static const char* _pitchFactName;
    static const char* _yawFactName;
    static const char* _axFactName;
    static const char* _ayFactName;
    static const char* _azFactName;
    static const char* _gxFactName;
    static const char* _gyFactName;
    static const char* _gzFactName;
    static const char* _mxFactName;
    static const char* _myFactName;
    static const char* _mzFactName;
    static const char* _vnsFactName;
    static const char* _vewFactName;
    static const char* _vudFactName;

private:
    Fact        _rollFact;
    Fact        _pitchFact;
    Fact        _yawFact;
    Fact        _axFact;
    Fact        _ayFact;
    Fact        _azFact;
    Fact        _gxFact;
    Fact        _gyFact;
    Fact        _gzFact;
    Fact        _mxFact;
    Fact        _myFact;
    Fact        _mzFact;
    Fact        _vnsFact;
    Fact        _vewFact;
    Fact        _vudFact;
};

class VehicleMhpFactGroup : public FactGroup
{
    Q_OBJECT

public:
    VehicleMhpFactGroup(QObject* parent = NULL);
    Q_PROPERTY(Fact* dp0                  READ dp0                 CONSTANT)
    Q_PROPERTY(Fact* dp1                  READ dp1                 CONSTANT)
    Q_PROPERTY(Fact* dp2                  READ dp2                 CONSTANT)
    Q_PROPERTY(Fact* dp3                  READ dp3                 CONSTANT)
    Q_PROPERTY(Fact* dp4                  READ dp4                 CONSTANT)
    Q_PROPERTY(Fact* dpS                  READ dpS                 CONSTANT)
    Q_PROPERTY(Fact* tas                  READ tas                 CONSTANT)
    Q_PROPERTY(Fact* aoa                  READ aoa                 CONSTANT)
    Q_PROPERTY(Fact* aos                  READ aos                 CONSTANT)
    Fact* dp0            (void) { return &_dp0Fact; }
    Fact* dp1            (void) { return &_dp1Fact; }
    Fact* dp2            (void) { return &_dp2Fact; }
    Fact* dp3            (void) { return &_dp3Fact; }
    Fact* dp4            (void) { return &_dp4Fact; }
    Fact* dpS            (void) { return &_dpSFact; }
    Fact* tas            (void) { return &_tasFact; }
    Fact* aoa            (void) { return &_aoaFact; }
    Fact* aos            (void) { return &_aosFact; }

    static const char* _dp0FactName;
    static const char* _dp1FactName;
    static const char* _dp2FactName;
    static const char* _dp3FactName;
    static const char* _dp4FactName;
    static const char* _dpSFactName;
    static const char* _tasFactName;
    static const char* _aoaFactName;
    static const char* _aosFactName;

private:
    Fact        _dp0Fact;
    Fact        _dp1Fact;
    Fact        _dp2Fact;
    Fact        _dp3Fact;
    Fact        _dp4Fact;
    Fact        _dpSFact;
    Fact        _tasFact;
    Fact        _aoaFact;
    Fact        _aosFact;
};

class VehicleSetpointFactGroup : public FactGroup
{
    Q_OBJECT

public:
    VehicleSetpointFactGroup(QObject* parent = NULL);

    Q_PROPERTY(Fact* roll       READ roll       CONSTANT)
    Q_PROPERTY(Fact* pitch      READ pitch      CONSTANT)
    Q_PROPERTY(Fact* yaw        READ yaw        CONSTANT)
    Q_PROPERTY(Fact* rollRate   READ rollRate   CONSTANT)
    Q_PROPERTY(Fact* pitchRate  READ pitchRate  CONSTANT)
    Q_PROPERTY(Fact* yawRate    READ yawRate    CONSTANT)

    Fact* roll      (void) { return &_rollFact; }
    Fact* pitch     (void) { return &_pitchFact; }
    Fact* yaw       (void) { return &_yawFact; }
    Fact* rollRate  (void) { return &_rollRateFact; }
    Fact* pitchRate (void) { return &_pitchRateFact; }
    Fact* yawRate   (void) { return &_yawRateFact; }

    static const char* _rollFactName;
    static const char* _pitchFactName;
    static const char* _yawFactName;
    static const char* _rollRateFactName;
    static const char* _pitchRateFactName;
    static const char* _yawRateFactName;

private:
    Fact _rollFact;
    Fact _pitchFact;
    Fact _yawFact;
    Fact _rollRateFact;
    Fact _pitchRateFact;
    Fact _yawRateFact;
};

class VehicleVibrationFactGroup : public FactGroup
{
    Q_OBJECT

public:
    VehicleVibrationFactGroup(QObject* parent = NULL);

    Q_PROPERTY(Fact* xAxis      READ xAxis      CONSTANT)
    Q_PROPERTY(Fact* yAxis      READ yAxis      CONSTANT)
    Q_PROPERTY(Fact* zAxis      READ zAxis      CONSTANT)
    Q_PROPERTY(Fact* clipCount1 READ clipCount1 CONSTANT)
    Q_PROPERTY(Fact* clipCount2 READ clipCount2 CONSTANT)
    Q_PROPERTY(Fact* clipCount3 READ clipCount3 CONSTANT)

    Fact* xAxis         (void) { return &_xAxisFact; }
    Fact* yAxis         (void) { return &_yAxisFact; }
    Fact* zAxis         (void) { return &_zAxisFact; }
    Fact* clipCount1    (void) { return &_clipCount1Fact; }
    Fact* clipCount2    (void) { return &_clipCount2Fact; }
    Fact* clipCount3    (void) { return &_clipCount3Fact; }

    static const char* _xAxisFactName;
    static const char* _yAxisFactName;
    static const char* _zAxisFactName;
    static const char* _clipCount1FactName;
    static const char* _clipCount2FactName;
    static const char* _clipCount3FactName;

private:
    Fact        _xAxisFact;
    Fact        _yAxisFact;
    Fact        _zAxisFact;
    Fact        _clipCount1Fact;
    Fact        _clipCount2Fact;
    Fact        _clipCount3Fact;
};

class VehicleWindFactGroup : public FactGroup
{
    Q_OBJECT

public:
    VehicleWindFactGroup(QObject* parent = NULL);

    Q_PROPERTY(Fact* direction      READ direction      CONSTANT)
    Q_PROPERTY(Fact* speed          READ speed          CONSTANT)
    Q_PROPERTY(Fact* verticalSpeed  READ verticalSpeed  CONSTANT)

    Fact* direction     (void) { return &_directionFact; }
    Fact* speed         (void) { return &_speedFact; }
    Fact* verticalSpeed (void) { return &_verticalSpeedFact; }

    static const char* _directionFactName;
    static const char* _speedFactName;
    static const char* _verticalSpeedFactName;

private:
    Fact        _directionFact;
    Fact        _speedFact;
    Fact        _verticalSpeedFact;
};

class VehicleGPSFactGroup : public FactGroup
{
    Q_OBJECT

public:
    VehicleGPSFactGroup(QObject* parent = NULL);

    Q_PROPERTY(Fact* lat                READ lat                CONSTANT)
    Q_PROPERTY(Fact* lon                READ lon                CONSTANT)
    Q_PROPERTY(Fact* hdop               READ hdop               CONSTANT)
    Q_PROPERTY(Fact* vdop               READ vdop               CONSTANT)
    Q_PROPERTY(Fact* courseOverGround   READ courseOverGround   CONSTANT)
    Q_PROPERTY(Fact* count              READ count              CONSTANT)
    Q_PROPERTY(Fact* lock               READ lock               CONSTANT)

    Fact* lat               (void) { return &_latFact; }
    Fact* lon               (void) { return &_lonFact; }
    Fact* hdop              (void) { return &_hdopFact; }
    Fact* vdop              (void) { return &_vdopFact; }
    Fact* courseOverGround  (void) { return &_courseOverGroundFact; }
    Fact* count             (void) { return &_countFact; }
    Fact* lock              (void) { return &_lockFact; }

    static const char* _latFactName;
    static const char* _lonFactName;
    static const char* _hdopFactName;
    static const char* _vdopFactName;
    static const char* _courseOverGroundFactName;
    static const char* _countFactName;
    static const char* _lockFactName;

private:
    Fact        _latFact;
    Fact        _lonFact;
    Fact        _hdopFact;
    Fact        _vdopFact;
    Fact        _courseOverGroundFact;
    Fact        _countFact;
    Fact        _lockFact;
};

class VehicleBatteryFactGroup : public FactGroup
{
    Q_OBJECT

public:
    VehicleBatteryFactGroup(QObject* parent = NULL);

    Q_PROPERTY(Fact* voltage            READ voltage            CONSTANT)
    Q_PROPERTY(Fact* percentRemaining   READ percentRemaining   CONSTANT)
    Q_PROPERTY(Fact* mahConsumed        READ mahConsumed        CONSTANT)
    Q_PROPERTY(Fact* current            READ current            CONSTANT)
    Q_PROPERTY(Fact* temperature        READ temperature        CONSTANT)
    Q_PROPERTY(Fact* cellCount          READ cellCount          CONSTANT)
    Q_PROPERTY(Fact* instantPower       READ instantPower       CONSTANT)
    Q_PROPERTY(Fact* timeRemaining      READ timeRemaining      CONSTANT)
    Q_PROPERTY(Fact* chargeState        READ chargeState        CONSTANT)

    Fact* voltage                   (void) { return &_voltageFact; }
    Fact* percentRemaining          (void) { return &_percentRemainingFact; }
    Fact* mahConsumed               (void) { return &_mahConsumedFact; }
    Fact* current                   (void) { return &_currentFact; }
    Fact* temperature               (void) { return &_temperatureFact; }
    Fact* cellCount                 (void) { return &_cellCountFact; }
    Fact* instantPower              (void) { return &_instantPowerFact; }
    Fact* timeRemaining             (void) { return &_timeRemainingFact; }
    Fact* chargeState               (void) { return &_chargeStateFact; }

    static const char* _voltageFactName;
    static const char* _percentRemainingFactName;
    static const char* _mahConsumedFactName;
    static const char* _currentFactName;
    static const char* _temperatureFactName;
    static const char* _cellCountFactName;
    static const char* _instantPowerFactName;
    static const char* _timeRemainingFactName;
    static const char* _chargeStateFactName;

    static const char* _settingsGroup;

    static const double _voltageUnavailable;
    static const int    _percentRemainingUnavailable;
    static const int    _mahConsumedUnavailable;
    static const int    _currentUnavailable;
    static const double _temperatureUnavailable;
    static const int    _cellCountUnavailable;
    static const double _instantPowerUnavailable;

private:
    Fact            _voltageFact;
    Fact            _percentRemainingFact;
    Fact            _mahConsumedFact;
    Fact            _currentFact;
    Fact            _temperatureFact;
    Fact            _cellCountFact;
    Fact            _instantPowerFact;
    Fact            _timeRemainingFact;
    Fact            _chargeStateFact;
};

class VehicleTemperatureFactGroup : public FactGroup
{
    Q_OBJECT

public:
    VehicleTemperatureFactGroup(QObject* parent = NULL);

    Q_PROPERTY(Fact* temperature1       READ temperature1       CONSTANT)
    Q_PROPERTY(Fact* temperature2       READ temperature2       CONSTANT)
    Q_PROPERTY(Fact* temperature3       READ temperature3       CONSTANT)

    Fact* temperature1 (void) { return &_temperature1Fact; }
    Fact* temperature2 (void) { return &_temperature2Fact; }
    Fact* temperature3 (void) { return &_temperature3Fact; }

    static const char* _temperature1FactName;
    static const char* _temperature2FactName;
    static const char* _temperature3FactName;

    static const char* _settingsGroup;

    static const double _temperatureUnavailable;

private:
    Fact            _temperature1Fact;
    Fact            _temperature2Fact;
    Fact            _temperature3Fact;
};

class VehicleClockFactGroup : public FactGroup
{
    Q_OBJECT

public:
    VehicleClockFactGroup(QObject* parent = NULL);

    Q_PROPERTY(Fact* currentTime        READ currentTime        CONSTANT)
    Q_PROPERTY(Fact* currentDate        READ currentDate        CONSTANT)

    Fact* currentTime (void) { return &_currentTimeFact; }
    Fact* currentDate (void) { return &_currentDateFact; }

    static const char* _currentTimeFactName;
    static const char* _currentDateFactName;

    static const char* _settingsGroup;

private slots:
    void _updateAllValues(void) override;

private:
    Fact            _currentTimeFact;
    Fact            _currentDateFact;
};

class Vehicle : public FactGroup
{
    Q_OBJECT

public:
    Vehicle(LinkInterface*          link,
            int                     vehicleId,
            int                     defaultComponentId,
            MAV_AUTOPILOT           firmwareType,
            MAV_TYPE                vehicleType,
            FirmwarePluginManager*  firmwarePluginManager,
            JoystickManager*        joystickManager);

    // The following is used to create a disconnected Vehicle for use while offline editing.
    Vehicle(MAV_AUTOPILOT           firmwareType,
            MAV_TYPE                vehicleType,
            FirmwarePluginManager*  firmwarePluginManager,
            QObject*                parent = NULL);

    ~Vehicle();

    /// Sensor bits from sensors*Bits properties
    enum MavlinkSysStatus {
        SysStatusSensor3dGyro =                 MAV_SYS_STATUS_SENSOR_3D_GYRO,
        SysStatusSensor3dAccel =                MAV_SYS_STATUS_SENSOR_3D_ACCEL,
        SysStatusSensor3dMag =                  MAV_SYS_STATUS_SENSOR_3D_MAG,
        SysStatusSensorAbsolutePressure =       MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE,
        SysStatusSensorDifferentialPressure =   MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE,
        SysStatusSensorGPS =                    MAV_SYS_STATUS_SENSOR_GPS,
        SysStatusSensorOpticalFlow =            MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW,
        SysStatusSensorVisionPosition =         MAV_SYS_STATUS_SENSOR_VISION_POSITION,
        SysStatusSensorLaserPosition =          MAV_SYS_STATUS_SENSOR_LASER_POSITION,
        SysStatusSensorExternalGroundTruth =    MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH,
        SysStatusSensorAngularRateControl =     MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL,
        SysStatusSensorAttitudeStabilization =  MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION,
        SysStatusSensorYawPosition =            MAV_SYS_STATUS_SENSOR_YAW_POSITION,
        SysStatusSensorZAltitudeControl =       MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL,
        SysStatusSensorXYPositionControl =      MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL,
        SysStatusSensorMotorOutputs =           MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS,
        SysStatusSensorRCReceiver =             MAV_SYS_STATUS_SENSOR_RC_RECEIVER,
        SysStatusSensor3dGyro2 =                MAV_SYS_STATUS_SENSOR_3D_GYRO2,
        SysStatusSensor3dAccel2 =               MAV_SYS_STATUS_SENSOR_3D_ACCEL2,
        SysStatusSensor3dMag2 =                 MAV_SYS_STATUS_SENSOR_3D_MAG2,
        SysStatusSensorGeoFence =               MAV_SYS_STATUS_GEOFENCE,
        SysStatusSensorAHRS =                   MAV_SYS_STATUS_AHRS,
        SysStatusSensorTerrain =                MAV_SYS_STATUS_TERRAIN,
        SysStatusSensorReverseMotor =           MAV_SYS_STATUS_REVERSE_MOTOR,
        SysStatusSensorLogging =                MAV_SYS_STATUS_LOGGING,
        SysStatusSensorBattery =                MAV_SYS_STATUS_SENSOR_BATTERY,
    };
    Q_ENUM(MavlinkSysStatus)

    Q_PROPERTY(int                  id                      READ id                                                     CONSTANT)
    Q_PROPERTY(AutoPilotPlugin*     autopilot               MEMBER _autopilotPlugin                                     CONSTANT)
    Q_PROPERTY(QGeoCoordinate       coordinate              READ coordinate                                             NOTIFY coordinateChanged)
    Q_PROPERTY(QGeoCoordinate       homePosition            READ homePosition                                           NOTIFY homePositionChanged)
    Q_PROPERTY(bool                 armed                   READ armed                  WRITE setArmed                  NOTIFY armedChanged)
    Q_PROPERTY(bool                 autoDisarm              READ autoDisarm                                             NOTIFY autoDisarmChanged)
    Q_PROPERTY(bool                 flightModeSetAvailable  READ flightModeSetAvailable                                 CONSTANT)
    Q_PROPERTY(QStringList          flightModes             READ flightModes                                            NOTIFY flightModesChanged)
    Q_PROPERTY(QString              flightMode              READ flightMode             WRITE setFlightMode             NOTIFY flightModeChanged)
    Q_PROPERTY(bool                 hilMode                 READ hilMode                WRITE setHilMode                NOTIFY hilModeChanged)
    Q_PROPERTY(QmlObjectListModel*  trajectoryPoints        READ trajectoryPoints                                       CONSTANT)
    Q_PROPERTY(QmlObjectListModel*  cameraTriggerPoints     READ cameraTriggerPoints                                    CONSTANT)
    Q_PROPERTY(float                latitude                READ latitude                                               NOTIFY coordinateChanged)
    Q_PROPERTY(float                longitude               READ longitude                                              NOTIFY coordinateChanged)
    Q_PROPERTY(bool                 messageTypeNone         READ messageTypeNone                                        NOTIFY messageTypeChanged)
    Q_PROPERTY(bool                 messageTypeNormal       READ messageTypeNormal                                      NOTIFY messageTypeChanged)
    Q_PROPERTY(bool                 messageTypeWarning      READ messageTypeWarning                                     NOTIFY messageTypeChanged)
    Q_PROPERTY(bool                 messageTypeError        READ messageTypeError                                       NOTIFY messageTypeChanged)
    Q_PROPERTY(int                  newMessageCount         READ newMessageCount                                        NOTIFY newMessageCountChanged)
    Q_PROPERTY(int                  messageCount            READ messageCount                                           NOTIFY messageCountChanged)
    Q_PROPERTY(QString              formatedMessages        READ formatedMessages                                       NOTIFY formatedMessagesChanged)
    Q_PROPERTY(QString              formatedMessage         READ formatedMessage                                        NOTIFY formatedMessageChanged)
    Q_PROPERTY(QString              latestError             READ latestError                                            NOTIFY latestErrorChanged)
    Q_PROPERTY(int                  joystickMode            READ joystickMode           WRITE setJoystickMode           NOTIFY joystickModeChanged)
    Q_PROPERTY(QStringList          joystickModes           READ joystickModes                                          CONSTANT)
    Q_PROPERTY(bool                 joystickEnabled         READ joystickEnabled        WRITE setJoystickEnabled        NOTIFY joystickEnabledChanged)
    Q_PROPERTY(bool                 active                  READ active                 WRITE setActive                 NOTIFY activeChanged)
    Q_PROPERTY(int                  flowImageIndex          READ flowImageIndex                                         NOTIFY flowImageIndexChanged)
    Q_PROPERTY(int                  rcRSSI                  READ rcRSSI                                                 NOTIFY rcRSSIChanged)
    Q_PROPERTY(bool                 px4Firmware             READ px4Firmware                                            NOTIFY firmwareTypeChanged)
    Q_PROPERTY(bool                 apmFirmware             READ apmFirmware                                            NOTIFY firmwareTypeChanged)
    Q_PROPERTY(bool                 soloFirmware            READ soloFirmware           WRITE setSoloFirmware           NOTIFY soloFirmwareChanged)
    Q_PROPERTY(bool                 genericFirmware         READ genericFirmware                                        CONSTANT)
    Q_PROPERTY(bool                 connectionLost          READ connectionLost                                         NOTIFY connectionLostChanged)
    Q_PROPERTY(bool                 connectionLostEnabled   READ connectionLostEnabled  WRITE setConnectionLostEnabled  NOTIFY connectionLostEnabledChanged)
    Q_PROPERTY(uint                 messagesReceived        READ messagesReceived                                       NOTIFY messagesReceivedChanged)
    Q_PROPERTY(uint                 messagesSent            READ messagesSent                                           NOTIFY messagesSentChanged)
    Q_PROPERTY(uint                 messagesLost            READ messagesLost                                           NOTIFY messagesLostChanged)
    Q_PROPERTY(bool                 fixedWing               READ fixedWing                                              NOTIFY vehicleTypeChanged)
    Q_PROPERTY(bool                 multiRotor              READ multiRotor                                             NOTIFY vehicleTypeChanged)
    Q_PROPERTY(bool                 vtol                    READ vtol                                                   NOTIFY vehicleTypeChanged)
    Q_PROPERTY(bool                 rover                   READ rover                                                  NOTIFY vehicleTypeChanged)
    Q_PROPERTY(bool                 sub                     READ sub                                                    NOTIFY vehicleTypeChanged)
    Q_PROPERTY(bool        supportsThrottleModeCenterZero   READ supportsThrottleModeCenterZero                         CONSTANT)
    Q_PROPERTY(bool                supportsNegativeThrust   READ supportsNegativeThrust                                 CONSTANT)
    Q_PROPERTY(bool                 supportsJSButton        READ supportsJSButton                                       CONSTANT)
    Q_PROPERTY(bool                 supportsRadio           READ supportsRadio                                          CONSTANT)
    Q_PROPERTY(bool               supportsMotorInterference READ supportsMotorInterference                              CONSTANT)
    Q_PROPERTY(bool                 autoDisconnect          MEMBER _autoDisconnect                                      NOTIFY autoDisconnectChanged)
    Q_PROPERTY(QString              prearmError             READ prearmError            WRITE setPrearmError            NOTIFY prearmErrorChanged)
    Q_PROPERTY(int                  motorCount              READ motorCount                                             CONSTANT)
    Q_PROPERTY(bool                 coaxialMotors           READ coaxialMotors                                          CONSTANT)
    Q_PROPERTY(bool                 xConfigMotors           READ xConfigMotors                                          CONSTANT)
    Q_PROPERTY(bool                 isOfflineEditingVehicle READ isOfflineEditingVehicle                                CONSTANT)
    Q_PROPERTY(QString              brandImageIndoor        READ brandImageIndoor                                       NOTIFY firmwareTypeChanged)
    Q_PROPERTY(QString              brandImageOutdoor       READ brandImageOutdoor                                      NOTIFY firmwareTypeChanged)
    Q_PROPERTY(QStringList          unhealthySensors        READ unhealthySensors                                       NOTIFY unhealthySensorsChanged)
    Q_PROPERTY(int                  sensorsPresentBits      READ sensorsPresentBits                                     NOTIFY sensorsPresentBitsChanged)
    Q_PROPERTY(int                  sensorsEnabledBits      READ sensorsEnabledBits                                     NOTIFY sensorsEnabledBitsChanged)
    Q_PROPERTY(int                  sensorsHealthBits       READ sensorsHealthBits                                      NOTIFY sensorsHealthBitsChanged)
    Q_PROPERTY(int                  sensorsUnhealthyBits    READ sensorsUnhealthyBits                                   NOTIFY sensorsUnhealthyBitsChanged) ///< Combination of enabled and health
    Q_PROPERTY(QString              missionFlightMode       READ missionFlightMode                                      CONSTANT)
    Q_PROPERTY(QString              pauseFlightMode         READ pauseFlightMode                                        CONSTANT)
    Q_PROPERTY(QString              rtlFlightMode           READ rtlFlightMode                                          CONSTANT)
    Q_PROPERTY(QString              landFlightMode          READ landFlightMode                                         CONSTANT)
    Q_PROPERTY(QString              takeControlFlightMode   READ takeControlFlightMode                                  CONSTANT)
    Q_PROPERTY(QString              firmwareTypeString      READ firmwareTypeString                                     NOTIFY firmwareTypeChanged)
    Q_PROPERTY(QString              vehicleTypeString       READ vehicleTypeString                                      NOTIFY vehicleTypeChanged)
    Q_PROPERTY(QString              vehicleImageOpaque      READ vehicleImageOpaque                                     CONSTANT)
    Q_PROPERTY(QString              vehicleImageOutline     READ vehicleImageOutline                                    CONSTANT)
    Q_PROPERTY(QString              vehicleImageCompass     READ vehicleImageCompass                                    CONSTANT)
    Q_PROPERTY(int                  telemetryRRSSI          READ telemetryRRSSI                                         NOTIFY telemetryRRSSIChanged)
    Q_PROPERTY(int                  telemetryLRSSI          READ telemetryLRSSI                                         NOTIFY telemetryLRSSIChanged)
    Q_PROPERTY(unsigned int         telemetryRXErrors       READ telemetryRXErrors                                      NOTIFY telemetryRXErrorsChanged)
    Q_PROPERTY(unsigned int         telemetryFixed          READ telemetryFixed                                         NOTIFY telemetryFixedChanged)
    Q_PROPERTY(unsigned int         telemetryTXBuffer       READ telemetryTXBuffer                                      NOTIFY telemetryTXBufferChanged)
    Q_PROPERTY(int                  telemetryLNoise         READ telemetryLNoise                                        NOTIFY telemetryLNoiseChanged)
    Q_PROPERTY(int                  telemetryRNoise         READ telemetryRNoise                                        NOTIFY telemetryRNoiseChanged)
    Q_PROPERTY(QVariantList         toolBarIndicators       READ toolBarIndicators                                      NOTIFY toolBarIndicatorsChanged)
    Q_PROPERTY(QmlObjectListModel*  adsbVehicles            READ adsbVehicles                                           CONSTANT)
    Q_PROPERTY(bool              initialPlanRequestComplete READ initialPlanRequestComplete                             NOTIFY initialPlanRequestCompleteChanged)
    Q_PROPERTY(QVariantList         staticCameraList        READ staticCameraList                                       CONSTANT)
    Q_PROPERTY(QGCCameraManager*    dynamicCameras          READ dynamicCameras                                         NOTIFY dynamicCamerasChanged)
    Q_PROPERTY(QString              hobbsMeter              READ hobbsMeter                                             NOTIFY hobbsMeterChanged)
    Q_PROPERTY(bool                 vtolInFwdFlight         READ vtolInFwdFlight        WRITE setVtolInFwdFlight        NOTIFY vtolInFwdFlightChanged)
    Q_PROPERTY(bool                 highLatencyLink         READ highLatencyLink                                        NOTIFY highLatencyLinkChanged)
    Q_PROPERTY(bool                 supportsTerrainFrame    READ supportsTerrainFrame                                   NOTIFY firmwareTypeChanged)
    Q_PROPERTY(QString              priorityLinkName        READ priorityLinkName       WRITE setPriorityLinkByName     NOTIFY priorityLinkNameChanged)
    Q_PROPERTY(QVariantList         links                   READ links                                                  NOTIFY linksChanged)
    Q_PROPERTY(LinkInterface*       priorityLink            READ priorityLink                                           NOTIFY priorityLinkNameChanged)

    // Vehicle state used for guided control
    Q_PROPERTY(bool flying                  READ flying NOTIFY flyingChanged)                               ///< Vehicle is flying
    Q_PROPERTY(bool landing                 READ landing NOTIFY landingChanged)                             ///< Vehicle is in landing pattern (DO_LAND_START)
    Q_PROPERTY(bool guidedMode              READ guidedMode WRITE setGuidedMode NOTIFY guidedModeChanged)   ///< Vehicle is in Guided mode and can respond to guided commands
    Q_PROPERTY(bool guidedModeSupported     READ guidedModeSupported CONSTANT)                              ///< Guided mode commands are supported by this vehicle
    Q_PROPERTY(bool pauseVehicleSupported   READ pauseVehicleSupported CONSTANT)                            ///< Pause vehicle command is supported
    Q_PROPERTY(bool orbitModeSupported      READ orbitModeSupported CONSTANT)                               ///< Orbit mode is supported by this vehicle
    Q_PROPERTY(bool takeoffVehicleSupported READ takeoffVehicleSupported CONSTANT)                          ///< Guided takeoff supported

    Q_PROPERTY(ParameterManager* parameterManager READ parameterManager CONSTANT)

    // FactGroup object model properties

    Q_PROPERTY(Fact* roll               READ roll               CONSTANT)
    Q_PROPERTY(Fact* pitch              READ pitch              CONSTANT)
    Q_PROPERTY(Fact* heading            READ heading            CONSTANT)
    Q_PROPERTY(Fact* rollRate           READ rollRate           CONSTANT)
    Q_PROPERTY(Fact* pitchRate          READ pitchRate          CONSTANT)
    Q_PROPERTY(Fact* yawRate            READ yawRate            CONSTANT)
    Q_PROPERTY(Fact* groundSpeed        READ groundSpeed        CONSTANT)
    Q_PROPERTY(Fact* airSpeed           READ airSpeed           CONSTANT)
    Q_PROPERTY(Fact* climbRate          READ climbRate          CONSTANT)
    Q_PROPERTY(Fact* altitudeRelative   READ altitudeRelative   CONSTANT)
    Q_PROPERTY(Fact* altitudeAMSL       READ altitudeAMSL       CONSTANT)
    Q_PROPERTY(Fact* flightDistance     READ flightDistance     CONSTANT)
    Q_PROPERTY(Fact* distanceToHome     READ distanceToHome     CONSTANT)
    Q_PROPERTY(Fact* hobbs              READ hobbs              CONSTANT)

    Q_PROPERTY(FactGroup* gps         READ gpsFactGroup         CONSTANT)
    Q_PROPERTY(FactGroup* battery     READ battery1FactGroup    CONSTANT)
    Q_PROPERTY(FactGroup* battery2    READ battery2FactGroup    CONSTANT)
    Q_PROPERTY(FactGroup* wind        READ windFactGroup        CONSTANT)
    Q_PROPERTY(FactGroup* vibration   READ vibrationFactGroup   CONSTANT)
    Q_PROPERTY(FactGroup* temperature READ temperatureFactGroup CONSTANT)
    Q_PROPERTY(FactGroup* clock       READ clockFactGroup       CONSTANT)
    Q_PROPERTY(FactGroup* setpoint    READ setpointFactGroup    CONSTANT)

    Q_PROPERTY(int      firmwareMajorVersion        READ firmwareMajorVersion       NOTIFY firmwareVersionChanged)
    Q_PROPERTY(int      firmwareMinorVersion        READ firmwareMinorVersion       NOTIFY firmwareVersionChanged)
    Q_PROPERTY(int      firmwarePatchVersion        READ firmwarePatchVersion       NOTIFY firmwareVersionChanged)
    Q_PROPERTY(int      firmwareVersionType         READ firmwareVersionType        NOTIFY firmwareVersionChanged)
    Q_PROPERTY(QString  firmwareVersionTypeString   READ firmwareVersionTypeString  NOTIFY firmwareVersionChanged)
    Q_PROPERTY(int      firmwareCustomMajorVersion  READ firmwareCustomMajorVersion NOTIFY firmwareCustomVersionChanged)
    Q_PROPERTY(int      firmwareCustomMinorVersion  READ firmwareCustomMinorVersion NOTIFY firmwareCustomVersionChanged)
    Q_PROPERTY(int      firmwareCustomPatchVersion  READ firmwareCustomPatchVersion NOTIFY firmwareCustomVersionChanged)
    Q_PROPERTY(QString  gitHash                     READ gitHash                    NOTIFY gitHashChanged)
    Q_PROPERTY(quint64  vehicleUID                  READ vehicleUID                 NOTIFY vehicleUIDChanged)
    Q_PROPERTY(QString  vehicleUIDStr               READ vehicleUIDStr              NOTIFY vehicleUIDChanged)

    /// Resets link status counters
    Q_INVOKABLE void resetCounters  ();

    /// Returns the number of buttons which are reserved for firmware use in the MANUAL_CONTROL mavlink
    /// message. For example PX4 Flight Stack reserves the first 8 buttons to simulate rc switches.
    /// The remainder can be assigned to Vehicle actions.
    /// @return -1: reserver all buttons, >0 number of buttons to reserve
    Q_PROPERTY(int manualControlReservedButtonCount READ manualControlReservedButtonCount CONSTANT)

    // Called when the message drop-down is invoked to clear current count
    Q_INVOKABLE void        resetMessages();

    Q_INVOKABLE void virtualTabletJoystickValue(double roll, double pitch, double yaw, double thrust);
    Q_INVOKABLE void disconnectInactiveVehicle(void);

    /// Command vehicle to return to launch
    Q_INVOKABLE void guidedModeRTL(void);

    /// Command vehicle to land at current location
    Q_INVOKABLE void guidedModeLand(void);

    /// Command vehicle to takeoff from current location
    Q_INVOKABLE void guidedModeTakeoff(double altitudeRelative);

    /// @return The minimum takeoff altitude (relative) for guided takeoff.
    Q_INVOKABLE double minimumTakeoffAltitude(void);

    /// Command vehicle to move to specified location (altitude is included and relative)
    Q_INVOKABLE void guidedModeGotoLocation(const QGeoCoordinate& gotoCoord);

    /// Command vehicle to change altitude
    ///     @param altitudeChange If > 0, go up by amount specified, if < 0, go down by amount specified
    Q_INVOKABLE void guidedModeChangeAltitude(double altitudeChange);

    /// Command vehicle to orbit given center point
    ///     @param centerCoord Center Coordinates
    ///     @param radius Distance from vehicle to centerCoord
    ///     @param velocity Orbit velocity (positive CW, negative CCW)
    ///     @param altitude Desired Vehicle Altitude
    Q_INVOKABLE void guidedModeOrbit(const QGeoCoordinate& centerCoord = QGeoCoordinate(), double radius = NAN, double velocity = NAN, double altitude = NAN);

    /// Command vehicle to pause at current location. If vehicle supports guide mode, vehicle will be left
    /// in guided mode after pause.
    Q_INVOKABLE void pauseVehicle(void);

    /// Command vehicle to kill all motors no matter what state
    Q_INVOKABLE void emergencyStop(void);

    /// Command vehicle to abort landing
    Q_INVOKABLE void abortLanding(double climbOutAltitude);

    Q_INVOKABLE void startMission(void);

    /// Alter the current mission item on the vehicle
    Q_INVOKABLE void setCurrentMissionSequence(int seq);

    /// Reboot vehicle
    Q_INVOKABLE void rebootVehicle();

    /// Clear Messages
    Q_INVOKABLE void clearMessages();

    Q_INVOKABLE void triggerCamera(void);
    Q_INVOKABLE void sendPlan(QString planFile);

#if 0
    // Temporarily removed, waiting for new command implementation
    /// Test motor
    ///     @param motor Motor number, 1-based
    ///     @param percent 0-no power, 100-full power
    ///     @param timeoutSecs Number of seconds for motor to run
    Q_INVOKABLE void motorTest(int motor, int percent, int timeoutSecs);
#endif

    bool guidedModeSupported    (void) const;
    bool pauseVehicleSupported  (void) const;
    bool orbitModeSupported     (void) const;
    bool takeoffVehicleSupported(void) const;

    // Property accessors

    QGeoCoordinate coordinate(void) { return _coordinate; }

    typedef enum {
        JoystickModeRC,         ///< Joystick emulates an RC Transmitter
        JoystickModeAttitude,
        JoystickModePosition,
        JoystickModeForce,
        JoystickModeVelocity,
        JoystickModeMax
    } JoystickMode_t;

    int joystickMode(void);
    void setJoystickMode(int mode);

    /// List of joystick mode names
    QStringList joystickModes(void);

    bool joystickEnabled(void);
    void setJoystickEnabled(bool enabled);

    // Is vehicle active with respect to current active vehicle in QGC
    bool active(void);
    void setActive(bool active);

    // Property accesors
    int id(void) { return _id; }
    MAV_AUTOPILOT firmwareType(void) const { return _firmwareType; }
    MAV_TYPE vehicleType(void) const { return _vehicleType; }
    Q_INVOKABLE QString vehicleTypeName(void) const;

    /// Returns the highest quality link available to the Vehicle. If you need to hold a reference to this link use
    /// LinkManager::sharedLinkInterfaceForGet to get QSharedPointer for link.
    LinkInterface* priorityLink(void) { return _priorityLink.data(); }

    /// Sends a message to the specified link
    /// @return true: message sent, false: Link no longer connected
    bool sendMessageOnLink(LinkInterface* link, mavlink_message_t message);

    /// Sends the specified messages multiple times to the vehicle in order to attempt to
    /// guarantee that it makes it to the vehicle.
    void sendMessageMultiple(mavlink_message_t message);

    /// Provides access to uas from vehicle. Temporary workaround until UAS is fully phased out.
    UAS* uas(void) { return _uas; }

    /// Provides access to uas from vehicle. Temporary workaround until AutoPilotPlugin is fully phased out.
    AutoPilotPlugin* autopilotPlugin(void) { return _autopilotPlugin; }

    /// Provides access to the Firmware Plugin for this Vehicle
    FirmwarePlugin* firmwarePlugin(void) { return _firmwarePlugin; }

    int manualControlReservedButtonCount(void);

    MissionManager*     missionManager(void)    { return _missionManager; }
    GeoFenceManager*    geoFenceManager(void)   { return _geoFenceManager; }
    RallyPointManager*  rallyPointManager(void) { return _rallyPointManager; }

    QGeoCoordinate homePosition(void);

    bool armed(void) { return _armed; }
    void setArmed(bool armed);

    bool flightModeSetAvailable(void);
    QStringList flightModes(void);
    QString flightMode(void) const;
    void setFlightMode(const QString& flightMode);

    QString priorityLinkName(void) const;
    QVariantList links(void) const;
    void setPriorityLinkByName(const QString& priorityLinkName);

    bool hilMode(void);
    void setHilMode(bool hilMode);

    bool fixedWing(void) const;
    bool multiRotor(void) const;
    bool vtol(void) const;
    bool rover(void) const;
    bool sub(void) const;

    bool supportsThrottleModeCenterZero (void) const;
    bool supportsNegativeThrust         (void) const;
    bool supportsRadio                  (void) const;
    bool supportsJSButton               (void) const;
    bool supportsMotorInterference      (void) const;
    bool supportsTerrainFrame           (void) const;

    void setGuidedMode(bool guidedMode);

    QString prearmError(void) const { return _prearmError; }
    void setPrearmError(const QString& prearmError);

    QmlObjectListModel* trajectoryPoints(void) { return &_mapTrajectoryList; }
    QmlObjectListModel* cameraTriggerPoints(void) { return &_cameraTriggerPoints; }
    QmlObjectListModel* adsbVehicles(void) { return &_adsbVehicles; }

    int  flowImageIndex() { return _flowImageIndex; }

    //-- Mavlink Logging
    void startMavlinkLog();
    void stopMavlinkLog();

    /// Requests the specified data stream from the vehicle
    ///     @param stream Stream which is being requested
    ///     @param rate Rate at which to send stream in Hz
    ///     @param sendMultiple Send multiple time to guarantee Vehicle reception
    void requestDataStream(MAV_DATA_STREAM stream, uint16_t rate, bool sendMultiple = true);

    typedef enum {
        MessageNone,
        MessageNormal,
        MessageWarning,
        MessageError
    } MessageType_t;

    bool            messageTypeNone         () { return _currentMessageType == MessageNone; }
    bool            messageTypeNormal       () { return _currentMessageType == MessageNormal; }
    bool            messageTypeWarning      () { return _currentMessageType == MessageWarning; }
    bool            messageTypeError        () { return _currentMessageType == MessageError; }
    int             newMessageCount         () { return _currentMessageCount; }
    int             messageCount            () { return _messageCount; }
    QString         formatedMessages        ();
    QString         formatedMessage         () { return _formatedMessage; }
    QString         latestError             () { return _latestError; }
    float           latitude                () { return _coordinate.latitude(); }
    float           longitude               () { return _coordinate.longitude(); }
    bool            mavPresent              () { return _mav != NULL; }
    int             rcRSSI                  () { return _rcRSSI; }
    bool            px4Firmware             () const { return _firmwareType == MAV_AUTOPILOT_PX4; }
    bool            apmFirmware             () const { return _firmwareType == MAV_AUTOPILOT_ARDUPILOTMEGA; }
    bool            genericFirmware         () const { return !px4Firmware() && !apmFirmware(); }
    bool            connectionLost          () const { return _connectionLost; }
    bool            connectionLostEnabled   () const { return _connectionLostEnabled; }
    uint            messagesReceived        () { return _messagesReceived; }
    uint            messagesSent            () { return _messagesSent; }
    uint            messagesLost            () { return _messagesLost; }
    bool            flying                  () const { return _flying; }
    bool            landing                 () const { return _landing; }
    bool            guidedMode              () const;
    bool            vtolInFwdFlight         () const { return _vtolInFwdFlight; }
    uint8_t         baseMode                () const { return _base_mode; }
    uint32_t        customMode              () const { return _custom_mode; }
    bool            isOfflineEditingVehicle () const { return _offlineEditingVehicle; }
    QString         brandImageIndoor        () const;
    QString         brandImageOutdoor       () const;
    QStringList     unhealthySensors        () const;
    int             sensorsPresentBits      () const { return _onboardControlSensorsPresent; }
    int             sensorsEnabledBits      () const { return _onboardControlSensorsEnabled; }
    int             sensorsHealthBits       () const { return _onboardControlSensorsHealth; }
    int             sensorsUnhealthyBits    () const { return _onboardControlSensorsUnhealthy; }
    QString         missionFlightMode       () const;
    QString         pauseFlightMode         () const;
    QString         rtlFlightMode           () const;
    QString         landFlightMode          () const;
    QString         takeControlFlightMode   () const;
    double          defaultCruiseSpeed      () const { return _defaultCruiseSpeed; }
    double          defaultHoverSpeed       () const { return _defaultHoverSpeed; }
    QString         firmwareTypeString      () const;
    QString         vehicleTypeString       () const;
    int             telemetryRRSSI          () { return _telemetryRRSSI; }
    int             telemetryLRSSI          () { return _telemetryLRSSI; }
    unsigned int    telemetryRXErrors       () { return _telemetryRXErrors; }
    unsigned int    telemetryFixed          () { return _telemetryFixed; }
    unsigned int    telemetryTXBuffer       () { return _telemetryTXBuffer; }
    int             telemetryLNoise         () { return _telemetryLNoise; }
    int             telemetryRNoise         () { return _telemetryRNoise; }
    bool            autoDisarm              ();
    bool            highLatencyLink         () const { return _highLatencyLink; }
    /// Get the maximum MAVLink protocol version supported
    /// @return the maximum version
    unsigned        maxProtoVersion         () const { return _maxProtoVersion; }

    Fact* roll              (void) { return &_rollFact; }
    Fact* pitch             (void) { return &_pitchFact; }
    Fact* heading           (void) { return &_headingFact; }
    Fact* rollRate          (void) { return &_rollRateFact; }
    Fact* pitchRate         (void) { return &_pitchRateFact; }
    Fact* yawRate           (void) { return &_yawRateFact; }
    Fact* airSpeed          (void) { return &_airSpeedFact; }
    Fact* groundSpeed       (void) { return &_groundSpeedFact; }
    Fact* climbRate         (void) { return &_climbRateFact; }
    Fact* vns               (void) { return &_vnsFact; }
    Fact* vew               (void) { return &_vewFact; }
    Fact* altitudeRelative  (void) { return &_altitudeRelativeFact; }
    Fact* altitudeAMSL      (void) { return &_altitudeAMSLFact; }
    Fact* flightDistance    (void) { return &_flightDistanceFact; }
    Fact* distanceToHome    (void) { return &_distanceToHomeFact; }
    Fact* hobbs             (void) { return &_hobbsFact; }
    Fact* temp              (void) { return &_tempFact; }
    Fact* hum               (void) { return &_humFact; }
    Fact* pt100             (void) { return &_pt100Fact; }
    Fact* aos               (void) { return &_aosFact; }
    Fact* aoa               (void) { return &_aoaFact; }
    Fact* windSpeed         (void) { return &_windSpeedFact; }
    Fact* windDir           (void) { return &_windDirFact; }
    Fact* windVert          (void) { return &_windVertFact; }

    FactGroup* gpsFactGroup             (void) { return &_gpsFactGroup; }
    FactGroup* battery1FactGroup        (void) { return &_battery1FactGroup; }
    FactGroup* battery2FactGroup        (void) { return &_battery2FactGroup; }
    FactGroup* windFactGroup            (void) { return &_windFactGroup; }
    FactGroup* vibrationFactGroup       (void) { return &_vibrationFactGroup; }
    FactGroup* temperatureFactGroup     (void) { return &_temperatureFactGroup; }
    FactGroup* clockFactGroup           (void) { return &_clockFactGroup; }
    FactGroup* setpointFactGroup        (void) { return &_setpointFactGroup; }
    FactGroup* distanceSensorFactGroup  (void) { return &_distanceSensorFactGroup; }

    void setConnectionLostEnabled(bool connectionLostEnabled);

    ParameterManager* parameterManager(void) { return _parameterManager; }
    ParameterManager* parameterManager(void) const { return _parameterManager; }

    static const int cMaxRcChannels = 18;

    bool containsLink(LinkInterface* link) { return _links.contains(link); }

    /// Sends the specified MAV_CMD to the vehicle. If no Ack is received command will be retried. If a sendMavCommand is already in progress
    /// the command will be queued and sent when the previous command completes.
    ///     @param component Component to send to
    ///     @param command MAV_CMD to send
    ///     @param showError true: Display error to user if command failed, false:  no error shown
    /// Signals: mavCommandResult on success or failure
    void sendMavCommand(int component, MAV_CMD command, bool showError, float param1 = 0.0f, float param2 = 0.0f, float param3 = 0.0f, float param4 = 0.0f, float param5 = 0.0f, float param6 = 0.0f, float param7 = 0.0f);

    /// Same as sendMavCommand but available from Qml.
    Q_INVOKABLE void sendCommand(int component, int command, bool showError, double param1 = 0.0f, double param2 = 0.0f, double param3 = 0.0f, double param4 = 0.0f, double param5 = 0.0f, double param6 = 0.0f, double param7 = 0.0f)
        { sendMavCommand(component, (MAV_CMD)command, showError, param1, param2, param3, param4, param5, param6, param7); }

    int firmwareMajorVersion(void) const { return _firmwareMajorVersion; }
    int firmwareMinorVersion(void) const { return _firmwareMinorVersion; }
    int firmwarePatchVersion(void) const { return _firmwarePatchVersion; }
    int firmwareVersionType(void) const { return _firmwareVersionType; }
    int firmwareCustomMajorVersion(void) const { return _firmwareCustomMajorVersion; }
    int firmwareCustomMinorVersion(void) const { return _firmwareCustomMinorVersion; }
    int firmwareCustomPatchVersion(void) const { return _firmwareCustomPatchVersion; }
    QString firmwareVersionTypeString(void) const;
    void setFirmwareVersion(int majorVersion, int minorVersion, int patchVersion, FIRMWARE_VERSION_TYPE versionType = FIRMWARE_VERSION_TYPE_OFFICIAL);
    void setFirmwareCustomVersion(int majorVersion, int minorVersion, int patchVersion);
    static const int versionNotSetValue = -1;

    QString gitHash(void) const { return _gitHash; }
    quint64 vehicleUID(void) const { return _uid; }
    QString vehicleUIDStr();

    bool soloFirmware(void) const { return _soloFirmware; }
    void setSoloFirmware(bool soloFirmware);

    int defaultComponentId(void) { return _defaultComponentId; }

    /// Sets the default component id for an offline editing vehicle
    void setOfflineEditingDefaultComponentId(int defaultComponentId);

    /// @return -1 = Unknown, Number of motors on vehicle
    int motorCount(void);

    /// @return true: Motors are coaxial like an X8 config, false: Quadcopter for example
    bool coaxialMotors(void);

    /// @return true: X confiuration, false: Plus configuration
    bool xConfigMotors(void);

    /// @return Firmware plugin instance data associated with this Vehicle
    QObject* firmwarePluginInstanceData(void) { return _firmwarePluginInstanceData; }

    /// Sets the firmware plugin instance data associated with this Vehicle. This object will be parented to the Vehicle
    /// and destroyed when the vehicle goes away.
    void setFirmwarePluginInstanceData(QObject* firmwarePluginInstanceData);

    QString vehicleImageOpaque  () const;
    QString vehicleImageOutline () const;
    QString vehicleImageCompass () const;

    const QVariantList&         toolBarIndicators   ();
    const QVariantList&         staticCameraList    (void) const;

    bool capabilitiesKnown      (void) const { return _vehicleCapabilitiesKnown; }
    uint64_t capabilityBits     (void) const { return _capabilityBits; }    // Change signalled by capabilityBitsChanged

    QGCCameraManager*           dynamicCameras      () { return _cameras; }
    QString                     hobbsMeter          ();

    /// @true: When flying a mission the vehicle is always facing towards the next waypoint
    bool vehicleYawsToNextWaypointInMission(void) const;

    /// The vehicle is responsible for making the initial request for the Plan.
    /// @return: true: initial request is complete, false: initial request is still in progress;
    bool initialPlanRequestComplete(void) const { return _initialPlanRequestComplete; }

    void forceInitialPlanRequestComplete(void);

    void _setFlying(bool flying);
    void _setLanding(bool landing);
    void setVtolInFwdFlight(bool vtolInFwdFlight);
    void _setHomePosition(QGeoCoordinate& homeCoord);
    void _setMaxProtoVersion (unsigned version);

    /// Vehicle is about to be deleted
    void prepareDelete();

signals:
    void allLinksInactive(Vehicle* vehicle);
    void coordinateChanged(QGeoCoordinate coordinate);
    void joystickModeChanged(int mode);
    void joystickEnabledChanged(bool enabled);
    void activeChanged(bool active);
    void mavlinkMessageReceived(const mavlink_message_t& message);
    void homePositionChanged(const QGeoCoordinate& homePosition);
    void armedChanged(bool armed);
    void flightModeChanged(const QString& flightMode);
    void hilModeChanged(bool hilMode);
    /** @brief HIL actuator controls (replaces HIL controls) */
    void hilActuatorControlsChanged(quint64 time, quint64 flags, float ctl_0, float ctl_1, float ctl_2, float ctl_3, float ctl_4, float ctl_5, float ctl_6, float ctl_7, float ctl_8, float ctl_9, float ctl_10, float ctl_11, float ctl_12, float ctl_13, float ctl_14, float ctl_15, quint8 mode);
    void connectionLostChanged(bool connectionLost);
    void connectionLostEnabledChanged(bool connectionLostEnabled);
    void autoDisconnectChanged(bool autoDisconnectChanged);
    void flyingChanged(bool flying);
    void landingChanged(bool landing);
    void guidedModeChanged(bool guidedMode);
    void vtolInFwdFlightChanged(bool vtolInFwdFlight);
    void prearmErrorChanged(const QString& prearmError);
    void soloFirmwareChanged(bool soloFirmware);
    void unhealthySensorsChanged(void);
    void defaultCruiseSpeedChanged(double cruiseSpeed);
    void defaultHoverSpeedChanged(double hoverSpeed);
    void firmwareTypeChanged(void);
    void vehicleTypeChanged(void);
    void dynamicCamerasChanged();
    void hobbsMeterChanged();
    void capabilitiesKnownChanged(bool capabilitiesKnown);
    void initialPlanRequestCompleteChanged(bool initialPlanRequestComplete);
    void capabilityBitsChanged(uint64_t capabilityBits);
    void toolBarIndicatorsChanged(void);
    void highLatencyLinkChanged(bool highLatencyLink);
    void priorityLinkNameChanged(const QString& priorityLinkName);
    void linksChanged(void);
    void linksPropertiesChanged(void);

    void messagesReceivedChanged    ();
    void messagesSentChanged        ();
    void messagesLostChanged        ();

    /// Used internally to move sendMessage call to main thread
    void _sendMessageOnLinkOnThread(LinkInterface* link, mavlink_message_t message);

    void messageTypeChanged         ();
    void newMessageCountChanged     ();
    void messageCountChanged        ();
    void formatedMessagesChanged    ();
    void formatedMessageChanged     ();
    void latestErrorChanged         ();
    void longitudeChanged           ();
    void currentConfigChanged       ();
    void flowImageIndexChanged      ();
    void rcRSSIChanged              (int rcRSSI);
    void telemetryRRSSIChanged      (int value);
    void telemetryLRSSIChanged      (int value);
    void telemetryRXErrorsChanged   (unsigned int value);
    void telemetryFixedChanged      (unsigned int value);
    void telemetryTXBufferChanged   (unsigned int value);
    void telemetryLNoiseChanged     (int value);
    void telemetryRNoiseChanged     (int value);
    void autoDisarmChanged          (void);
    void flightModesChanged         (void);
    void sensorsPresentBitsChanged  (int sensorsPresentBits);
    void sensorsEnabledBitsChanged  (int sensorsEnabledBits);
    void sensorsHealthBitsChanged   (int sensorsHealthBits);
    void sensorsUnhealthyBitsChanged(int sensorsUnhealthyBits);

    void firmwareVersionChanged(void);
    void firmwareCustomVersionChanged(void);
    void gitHashChanged(QString hash);
    void vehicleUIDChanged();

    /// New RC channel values
    ///     @param channelCount Number of available channels, cMaxRcChannels max
    ///     @param pwmValues -1 signals channel not available
    void rcChannelsChanged(int channelCount, int pwmValues[cMaxRcChannels]);

    /// Remote control RSSI changed  (0% - 100%)
    void remoteControlRSSIChanged(uint8_t rssi);

    void mavlinkRawImu(mavlink_message_t message);
    void mavlinkScaledImu1(mavlink_message_t message);
    void mavlinkScaledImu2(mavlink_message_t message);
    void mavlinkScaledImu3(mavlink_message_t message);

    // Mavlink Log Download
    void mavlinkLogData (Vehicle* vehicle, uint8_t target_system, uint8_t target_component, uint16_t sequence, uint8_t first_message, QByteArray data, bool acked);

    /// Signalled in response to usage of sendMavCommand
    ///     @param vehicleId Vehicle which command was sent to
    ///     @param component Component which command was sent to
    ///     @param command MAV_CMD Command which was sent
    ///     @param result MAV_RESULT returned in ack
    ///     @param noResponseFromVehicle true: vehicle did not respond to command, false: vehicle responsed, MAV_RESULT in result
    void mavCommandResult(int vehicleId, int component, int command, int result, bool noReponseFromVehicle);

    // MAVlink Serial Data
    void mavlinkSerialControl(uint8_t device, uint8_t flags, uint16_t timeout, uint32_t baudrate, QByteArray data);

    // MAVLink protocol version
    void requestProtocolVersion(unsigned version);

private slots:
    void _mavlinkMessageReceived(LinkInterface* link, mavlink_message_t message);
    void _linkInactiveOrDeleted(LinkInterface* link);
    void _sendMessageOnLink(LinkInterface* link, mavlink_message_t message);
    void _sendMessageMultipleNext(void);
    void _addNewMapTrajectoryPoint(void);
    void _parametersReady(bool parametersReady);
    void _remoteControlRSSIChanged(uint8_t rssi);
    void _handleFlightModeChanged(const QString& flightMode);
    void _announceArmedChanged(bool armed);
    void _offlineFirmwareTypeSettingChanged(QVariant value);
    void _offlineVehicleTypeSettingChanged(QVariant value);
    void _offlineCruiseSpeedSettingChanged(QVariant value);
    void _offlineHoverSpeedSettingChanged(QVariant value);
    void _updateHighLatencyLink(bool sendCommand = true);

    void _handleTextMessage                 (int newCount);
    void _handletextMessageReceived         (UASMessage* message);
    /** @brief A new camera image has arrived */
    void _imageReady                        (UASInterface* uas);
    void _prearmErrorTimeout(void);
    void _missionLoadComplete(void);
    void _geoFenceLoadComplete(void);
    void _rallyPointLoadComplete(void);
    void _sendMavCommandAgain(void);
    void _clearTrajectoryPoints(void);
    void _clearCameraTriggerPoints(void);
    void _updateDistanceToHome(void);
    void _updateHobbsMeter(void);
    void _vehicleParamLoaded(bool ready);
    void _sendQGCTimeToVehicle(void);

private:
    bool _containsLink(LinkInterface* link);
    void _addLink(LinkInterface* link);
    void _loadSettings(void);
    void _saveSettings(void);
    void _calcFlow(float _dp0,float _dp1,float _dp2,float _dp3,float _dp4,float _dpS);
    void _calcWind();
    void _startJoystick(bool start);
    void _handlePing(LinkInterface* link, mavlink_message_t& message);
    void _handleHomePosition(mavlink_message_t& message);
    void _handleHeartbeat(mavlink_message_t& message);
    void _handleRadioStatus(mavlink_message_t& message);
    void _handleRCChannels(mavlink_message_t& message);
    void _handleRCChannelsRaw(mavlink_message_t& message);
    void _handleBatteryStatus(mavlink_message_t& message);
    void _handleSysStatus(mavlink_message_t& message);
    void _handleWindCov(mavlink_message_t& message);
    void _handleVibration(mavlink_message_t& message);
    void _handleExtendedSysState(mavlink_message_t& message);
    void _handleCommandAck(mavlink_message_t& message);
    void _handleCommandLong(mavlink_message_t& message);
    void _handleAutopilotVersion(LinkInterface* link, mavlink_message_t& message);
    void _handleProtocolVersion(LinkInterface* link, mavlink_message_t& message);
    void _handleHilActuatorControls(mavlink_message_t& message);
    void _handleGpsRawInt(mavlink_message_t& message);
    void _handleGlobalPositionInt(mavlink_message_t& message);
    void _handleAltitude(mavlink_message_t& message);
    void _handleVfrHud(mavlink_message_t& message);
    void _handleScaledPressure(mavlink_message_t& message);
    void _handleScaledPressure2(mavlink_message_t& message);
    void _handleScaledPressure3(mavlink_message_t& message);
    void _handleHighLatency2(mavlink_message_t& message);
    void _handleAttitudeWorker(double rollRadians, double pitchRadians, double yawRadians);
    void _handleAttitude(mavlink_message_t& message);
    void _handleAttitudeQuaternion(mavlink_message_t& message);
    void _handleAttitudeTarget(mavlink_message_t& message);
    void _handleDistanceSensor(mavlink_message_t& message);
    void _handleMeteo(mavlink_message_t& message);
    void _handleIns(mavlink_message_t& message);
    void _handleMhp(mavlink_message_t& message);
    // ArduPilot dialect messages
#if !defined(NO_ARDUPILOT_DIALECT)
    void _handleCameraFeedback(const mavlink_message_t& message);
    void _handleWind(mavlink_message_t& message);
#endif
    void _handleCameraImageCaptured(const mavlink_message_t& message);
    void _handleADSBVehicle(const mavlink_message_t& message);
    void _missionManagerError(int errorCode, const QString& errorMsg);
    void _geoFenceManagerError(int errorCode, const QString& errorMsg);
    void _rallyPointManagerError(int errorCode, const QString& errorMsg);
    void _mapTrajectoryStart(void);
    void _mapTrajectoryStop(void);
    void _linkActiveChanged(LinkInterface* link, bool active, int vehicleID);
    void _say(const QString& text);
    QString _vehicleIdSpeech(void);
    void _handleMavlinkLoggingData(mavlink_message_t& message);
    void _handleMavlinkLoggingDataAcked(mavlink_message_t& message);
    void _ackMavlinkLogData(uint16_t sequence);
    void _sendNextQueuedMavCommand(void);
    void _updatePriorityLink(bool updateActive, bool sendCommand);
    void _commonInit(void);
    void _startPlanRequest(void);
    void _setupAutoDisarmSignalling(void);
    void _setCapabilities(uint64_t capabilityBits);
    void _updateArmed(bool armed);
    bool _apmArmingNotRequired(void);

    int     _id;                    ///< Mavlink system id
    int     _defaultComponentId;
    bool    _active;
    bool    _offlineEditingVehicle; ///< This Vehicle is a "disconnected" vehicle for ui use while offline editing

    const double cp_par = 1005.0; // specific heat capacity air (at const. pressure)
    const double cv_par = 717.0 ; // specific heat capacity air (at const. volume)

    const double poly_alpha_0025[144] = {0.7433144910154107,-0.7489778036892709,-1.7219808459257588,-0.29054658690058677,7.74633184719336,2.283390775187743,-13.345381060566368,-4.514408058551959,9.589485766624165,3.2636864866098967,-2.4003577647136414,-0.7679040785488835,-22.459493196654716,-5.0489836038070175,47.62638380028106,39.56221185905263,-161.33042185279334,-98.06752985754689,232.98505457080225,107.37511943152202,-146.27810824339429,-54.0748693068071,33.06056748555937,10.162806801457466,-6.0997227925092,-2.010562145335993,38.65434865817062,-4.3120473753288815,-104.7783923361014,62.762191670085556,157.45431319922182,-145.11400065546718,-115.49983841904282,121.35446903798788,30.38535566129758,-34.117170277413045,34.89978585477244,33.656187225302475,-330.1439838254666,-234.75547735001493,1123.8769653360775,544.5211738701296,-1696.2795318418641,-601.2171512490464,1089.154167222017,321.65347114959525,-243.6925306998619,-64.6873080256129,27.048867067202387,11.267666672268705,-143.39440104158257,4.139499277904996,485.463451692602,-257.38179248922347,-982.4154548603096,648.2813621957655,837.7385062552826,-542.2402347180584,-232.65188889307908,149.99922568669803,-94.43879865130647,-82.83644317003534,860.5328403549767,544.0237357346663,-3228.7625987716956,-1397.2220852489393,5112.236294634384,1927.5257304471857,-3070.68352448733,-1321.4826532712082,580.4633400239799,323.5687616346483,-44.168033837463746,-22.159748561253437,206.677473132185,7.049000194203693,-942.2734815673875,492.33616218213024,2131.0248447854638,-1427.4398903271094,-1783.9181665445442,1265.848755996472,455.3888768040563,-365.078542901494,113.74565111643632,90.60871115004123,-1023.2056181608195,-589.294823583326,4126.850326524889,1727.2844767141773,-5865.688711172981,-2848.718271550642,2195.368410744175,2153.2019871087264,31.586602421791092,-501.02640153766663,29.465126413473808,17.417386683726377,-127.89064229740508,-17.025127627661718,752.055839100473,-355.0419429003485,-1722.5418741652438,1105.6418594730783,1346.892641215041,-880.8618320399964,-290.5858740137001,212.92318691065572,-61.45215150509107,-45.682940546735374,558.3815418180303,298.47007213027473,-2291.029443090436,-961.3520216043594,2369.862251544622,1691.2592672485425,418.883666266545,-1118.8309770963465,-548.0215712824173,108.7583334479022,-6.879279776915894,-4.660546176438906,28.034908226140047,7.952823766379723,-201.86432574559473,78.13225249914727,438.18165478826234,-250.01075018477,-289.5616329899401,141.39188994462555,25.2526725364022,-23.371048723163298,12.283802944688496,8.655265175810115,-113.06633384047011,-55.850931454762204,447.12163410472874,182.97841884463944,-186.14341425573357,-299.00610878301717,-499.0145583155293,91.6317313337044,190.63872349995893,64.42293652146788};
    const double poly_beta_0025[144] = {-0.7752742072201976,22.17525523099208,5.239681651722339,-30.188885710001365,-23.18742188186473,84.5712397962536,39.282502915527395,-106.13103510327885,-27.330125460800982,59.6597678808405,6.658045218232935,-12.419139497695848,-1.461953039774384,3.161924003351183,15.770382784951806,-9.699699208227823,-80.39645025071407,18.770892902540147,139.41990156536934,-34.9590048850928,-99.59041059619645,32.893145235284415,25.14823580215063,-10.265440517332582,1.2306440121862519,-38.01882325950999,-30.860115543754247,201.43685583317037,124.57008082293271,-354.0516915523365,-186.1024171395343,254.53392429526622,115.20615420302876,-70.637865717424,-24.74546690397338,4.030114221390733,6.248257835900176,-22.912170921101715,-137.74018529784345,191.26131821271113,748.7321802762503,-1011.7469275177854,-1375.263474730635,2060.7935424291322,1049.1183669202248,-1664.674382110356,-281.8908028748603,453.233775755538,-2.191865578410976,113.13400825887189,69.9783606833308,-501.8160709032583,-324.8966549719514,788.8978365868506,628.1530771121787,-889.9628157385292,-521.1756048984595,710.8270319610962,148.73926402461126,-229.0534721073594,-15.061074410318646,66.89327941744463,401.50120619171287,-1017.3612988862158,-2251.274266023898,5450.667194608515,4085.7768526553564,-10130.740781973422,-3117.20350152572,7488.229887128236,830.9485978925619,-1862.3308079209153,0.3164675785743789,-149.1780537113475,-76.0626582735701,633.196317413079,496.7688457258622,-1409.5962323862102,-1278.1523169702884,2799.52345965926,1245.2423784258935,-2642.215656147548,-374.64360899966067,817.0348796672392,15.581048150578734,-97.48825019990925,-531.4540076607217,1937.708837227022,2985.1229231693583,-9789.394705797326,-5252.028371911962,16713.932225194236,4039.958895964036,-11406.126984126417,-1064.1002144664565,2553.819634474358,1.0766785671225634,88.52849100605184,46.389872429315254,-400.64355144693036,-428.3691109769432,1279.0749637175827,1293.100519421937,-3012.9705334551118,-1321.1672947934876,2717.4043164953,377.55490023330674,-762.2206732446103,-7.318121218562624,66.31452629934478,322.133885047592,-1467.4894341822308,-1747.688998197362,6927.37513126859,2856.0813617078516,-10787.974830232062,-2140.3660553346926,6664.765557898756,479.24152939609365,-1190.4812424867,-0.4342551438450357,-19.44862833969401,-12.063797459317101,97.49726252852759,133.92325992991073,-396.93870120590736,-413.33005573662024,968.0535463715551,377.33920117821657,-788.2230535447862,-61.74994640137901,193.1694281250768,1.2875690100473633,-16.45335346868646,-71.88327862491933,378.6587186264558,363.7134945938574,-1660.0402660179614,-520.0251353662597,2296.4210902877335,364.7317290802146,-1184.9118226769199,-38.26267264842067,53.485597612940424};
    const double poly_kp_0025[144] = {-0.10096459749062611,-0.10051309914960219,-0.10043865043268009,-0.103356831667015,-0.10441336974864743,-0.1039876018314048,-0.1043014090382711,-0.10405319951879041,-0.10432611548231155,-0.10645459986775756,-0.10575213302952215,-0.10506793450432779,-0.10539254681961573,-0.1054774146067581,-0.10257808881773363,-0.1036244660452922,-0.10488105004104586,-0.1051324163415969,-0.10553619979908405,-0.10291371415410547,-0.11146660688881442,-0.11047424503412134,-0.11153189916632651,-0.11052529796737615,-0.10490110462546687,-0.10537130037962437,-0.10574455820308262,-0.10296719819146055,-0.10209797357705923,-0.10199748910431203,-0.10279535703730405,-0.10148529981051257,-0.10368317155732279,-0.10193253474297279,-0.10394859358866892,-0.103121022350986,-0.10265714740310569,-0.10181743924860984,-0.10089076752859776,-0.10261341549883786,-0.1023930074062688,-0.10070166652195352,-0.10328182366704604,-0.10513687090965285,-0.10356436047581112,-0.10393137955697371,-0.10326980922265117,-0.10404704219159935,-0.10195028211615245,-0.10284333384738845,-0.10566891566754658,-0.10492071611247004,-0.10065996422640096,-0.10176961568323702,-0.10179191967932073,-0.10236212391008001,-0.10135742423223629,-0.10366316134720494,-0.10219832098373603,-0.1028722292842915,-0.10334893401048652,-0.10326649915407103,-0.10130942767474656,-0.10491010989110186,-0.10283523182731921,-0.10305212707731957,-0.10452723582606609,-0.10419428986829321,-0.10380176788323388,-0.10384737288719025,-0.1033985423859056,-0.10393077418753548,-0.10282655070544254,-0.102957732064248,-0.1028398493498403,-0.10374623561336281,-0.10451396350779202,-0.10426279994879378,-0.10249768732664043,-0.10593888121718006,-0.10354876712914275,-0.10441259567473639,-0.10520946018696797,-0.10410353859565723,-0.10369590342896859,-0.10437731259606159,-0.10508030556541961,-0.10132501549349175,-0.10574267647431167,-0.1037683121306055,-0.1051309615391922,-0.10246090096981916,-0.10115712514353128,-0.10265024262790574,-0.10557358166522118,-0.10019248095659741,-0.10303603946557031,-0.10313827881025912,-0.10172874728686553,-0.10309638207892728,-0.10142354201728593,-0.10173679498650924,-0.10017545755307601,-0.10124965514407408,-0.10326582927450678,-0.10356748993353142,-0.10533406003504274,-0.10400855597377055,-0.10460212358292764,-0.10792045699299126,-0.1052892050726831,-0.10634994885298447,-0.10498990481694767,-0.10707646995208052,-0.10723004887883159,-0.1047485457319622,-0.10753429591735274,-0.10842309548481088,-0.10793572413223845,-0.10695159682176278,-0.10819611413754229,-0.10495191691236734,-0.10552516747583678,-0.10589029466818171,-0.1066118512566211,-0.10906180616529215,-0.10435215963935886,-0.10341332230029288,-0.1041782271821761,-0.10631396339511857,-0.1067700535865167,-0.10465540169319905,-0.10549389624117475,-0.10190624871636836,-0.1040157290829429,-0.1044233239147235,-0.10460998367866246,-0.10413683005585503,-0.10682290894777238,-0.10834373667462002,-0.10577767731934114,-0.10683483928013633,-0.10464116335471853,-0.10568248752222778};
    const double poly_kq_0025[144] = {-0.013519635539798576,-0.0038546509540461965,-0.004693969581566176,-0.0052218519144111835,-0.000942830644129145,-0.00636221821179974,-0.012884381503861812,-0.008363421251144986,-0.005902265428578851,-8.185273657363935e-05,-0.0020087627148757344,-0.0013525421645203311,-0.006323039575589296,-0.0033761007065538197,-0.0009056759120770541,-0.002631863019682516,-0.003876127973870799,-0.003079449805015996,-0.007978085769565834,-0.005505103476458285,0.001243103070682751,-0.007662718435802865,-0.0049414468921033,-0.0043969913179636375,-0.008314054735991127,-0.008341193565798733,-0.010447433798157931,-0.0051550082337389605,-0.007134643395649666,-0.009941514498916377,-0.009518277334344202,-0.011002220818665675,-0.004377665722294635,-0.008836193702402599,-0.007715054588587095,-0.0069039254618966355,-0.0025868307730545015,-0.008842148253895236,-0.00709758671847002,-0.010671999265473789,-0.007640508959627864,-0.009628111633005611,-0.005686176250874142,-0.009168972523830216,-0.007855058861882359,-0.007515572347540556,-0.0034025283245587195,-0.00307331390310977,-0.00466105159872563,-0.002426859992842355,8.190595139352825e-05,-0.007652173913048178,-2.0559633101272712e-05,-0.0038879089118771465,-0.00455807698417737,-0.005993831690375439,-0.0035335144600621353,-0.0031773363672104684,-0.003694657115303448,-0.0036436809411621446,-0.003588326660771168,-0.004881800933316507,-0.010487687699982933,-0.008971129922983587,-0.008005840996184725,-0.003498667539764318,-0.012789262730569972,-0.007679036741547757,-0.00875757079704398,-0.008158925230298038,-0.0022298938754683286,-0.005400595293033181,-0.006358713488655864,-0.010172960802388624,0.0016030993252523179,-0.0070762796358484524,-0.008227673226263086,-0.004332092903775744,-0.0029396649194546537,2.053377553462423e-05,0.002397536324840843,-0.0040889710793675675,-0.0035888417780838206,-0.00365672025066981,-0.0034313599588221004,0.0011543654597196,-0.0054850312881552475,-0.004056483459190978,0.0010943347527270642,-0.0030150599165408246,-0.008009007586033432,0.0035960257713977513,-0.004671719120687002,-0.0024339484677789367,0.00016454895573702108,-0.006818740273492407,-0.0017078365004522423,-0.00800696257613888,-0.0036535116300428524,-0.00862387581620258,-0.011041379662372616,-0.007571300677926571,0.00041163062322182046,-0.010013589872080949,-0.0015175595999760176,-0.003918853068558953,-0.003599230792868521,0.001624812323895319,-0.0026300784401830624,-0.003815110710005116,0.0014846891433258603,-0.0005140410306185891,0.0009683134861942957,0.0006178973981730642,0.004749104403258464,-0.005817071661458573,-0.0019759388281049206,0.00012321973145240013,-0.00333312758481273,-0.001644618499907136,0.0009465166644117471,-0.00196948326698764,-0.007579947964724428,-0.00983931840728913,-0.0022668026747887818,-0.0001445758249263794,-0.004911656964969251,-0.0011377740999916782,-0.000390488521833843,-0.004188094724783872,0.0008259940323085732,-0.0036557251634324185,-0.0078850912226882,-0.0043547923257725146,-0.011305278356645646,-0.009000770614018112,-0.0045371026187314995,-0.002381795689186198,-0.006050725933878309,-0.004973054061410716,-0.004520633093980107,-0.0040295638409112156,-0.0046962620430654614,0.0017153899412398349};

    const double poly_alpha_0070[144] = {2.0106578102026087,-2.06346874811216,-7.596960801395177,4.890893841486456,36.176340340022364,-14.678508758211859,-72.33624478358358,24.219647148473463,63.20195047022823,-19.61139974924714,-19.815011455564097,6.010073419253576,-22.79816285074766,-10.110916908233975,52.71205156540029,119.0635857301963,-201.3594833390548,-476.34787214187895,366.8846772382642,840.0095832215162,-303.2676482185354,-666.2592480757231,91.49950424685167,193.8620077665403,-14.129693284688278,-11.821561870595435,170.8884728891763,125.98068016606382,-764.9197952086242,-477.6763645450775,1439.8605200910374,765.5949686953765,-1166.6077637335482,-538.8069037745756,335.69911862991836,135.63506429034487,43.74415425801641,97.39828670213748,-512.5104881220874,-1232.6167728546297,2145.8625590443717,4956.570128491593,-3941.1272150161158,-8651.853175978591,3163.6949347994523,6733.752745220707,-907.5386770304522,-1909.6931193895527,70.98622553690764,96.6182882580166,-919.5503191052242,-1037.6558816546437,3946.8565900698763,4061.713165833174,-6832.287463578691,-6697.50583310684,4873.029424289873,4837.698539572594,-1175.1910799000811,-1238.7356973830501,-148.07452205913356,-359.1930714109998,1894.4476520718313,4559.332206225991,-7938.871354197322,-18120.724516588947,14037.518432771485,31086.611745158512,-10437.528632000696,-23447.46255174537,2669.9988360908633,6337.476348941039,-148.08656542328168,-235.4726662334731,1898.5930176421518,2474.000693106975,-7562.095074432015,-9324.625545297691,11224.5814821105,13789.166716615162,-5941.438839691352,-8190.32255557097,743.2391581329521,1378.608169544966,244.9165829480777,602.0892385965537,-3120.570410729033,-7480.951674821801,12416.138436407351,28784.208927121712,-19480.28238793172,-46735.29946774023,11361.298438220358,31781.489864105246,-1762.39905926247,-7184.966205656344,133.44898854677348,229.1158620779571,-1640.8711693245318,-2312.4215616082106,5895.843690441437,8189.728105126683,-6788.405024082781,-10306.825616757715,1690.1964064213332,4809.674392148435,268.8517058727057,-311.6003611637658,-189.64113947204896,-462.92692377425556,2316.6097623916767,5539.671653530495,-8428.18967552867,-20215.122626138116,10642.520480354642,29746.098309274756,-3472.67146417003,-16714.922179487054,-201.88632153704842,2546.606143072112,-43.07661095495962,-77.24567897905813,498.1165699524001,739.4722814942515,-1569.474933261371,-2393.6620832244835,1144.734176793651,2244.2898878133383,240.07089897368826,-381.5093140574057,-102.60584525158552,-687.7034722961641,54.96218337014864,132.18624689418925,-630.080266631832,-1503.7973377638368,2024.6460878406037,5078.306774233963,-1722.00547987026,-6293.985126419025,-165.710484074612,2201.7821437685943,186.08524988094416,367.7331505935702};
    const double poly_beta_0070[144] = {-0.46498824981309145,24.25537287210905,8.238289084269876,-53.02864759813721,-43.73444107956556,183.05228004915216,92.23453963634054,-302.0896101478774,-83.08703993188355,231.62556480773654,26.665351440318794,-66.37502009254885,-3.8605181558029007,11.085704651254943,41.623237073341066,-107.67570453551645,-216.31280628948932,412.2906107696215,449.79324379543453,-708.1773170259921,-399.1903409323777,549.9169903376669,125.99182084953688,-156.90906349078958,0.4775732511446673,-58.863097993740936,-57.27440369961246,590.9728647038686,369.702877875782,-2223.0299698167178,-827.8753825067809,3734.453516740246,758.5471439779939,-2840.3676605445316,-242.3124155545632,795.0208323717745,18.83759274726458,-95.62735578477464,-391.4017588350496,1001.1222610871291,2104.6355720615143,-3814.5969439324504,-4357.634152939493,6361.250727992872,3784.073037168634,-4705.855604842025,-1154.5672004071125,1260.1140481987254,1.1150906726886802,222.17943747912295,163.09018543514756,-2393.8224947189165,-1212.376799287466,8881.171220408938,2787.004651636789,-14363.18781934939,-2564.0902902685534,10373.350426845209,813.481592255847,-2721.4741911449296,-59.087617701082536,290.3543915809985,1324.578974972405,-3004.730930718381,-6947.809240062797,11136.408866139109,13704.078776256525,-17424.32345447316,-11090.618951511215,11564.296564158318,3076.021617245775,-2643.3905046206173,-5.969241402146508,-392.79455598457037,-187.25051707599022,4128.389441047491,1684.2618126082145,-14439.619083073838,-3745.3777591615235,21439.87004724814,3322.346003344354,-13877.238011779596,-997.6076532909501,3128.5614620970505,87.50866146692158,-378.39524844933965,-2007.7557322024832,3798.658366557839,10052.385931456594,-13580.639129474755,-18644.315836094658,19188.750140840264,13727.477867942975,-10345.302189033697,-3317.8326740226003,1648.292364605743,7.485719572618564,313.4466154090407,74.61879123854749,-3095.0325571546946,-1016.0277409606401,9902.245499143673,2106.751547829537,-13047.321798189356,-2158.334818052169,7604.610823878423,773.4016716255974,-1562.2837580836901,-60.23627247121772,213.6901194761415,1389.7209026143732,-2085.7215768097426,-6529.851497443663,7172.307733096042,11196.121519820816,-8411.729852664015,-6830.646056226653,2294.5161401954074,1100.8512935922913,264.2207657299244,-2.8923521643612844,-91.5248048089212,-2.409540611310206,830.6731831198952,227.12671624182974,-2383.6333370157154,-437.888133660275,2783.2368824878076,719.8060051955093,-1706.5959007643933,-189.02674581723866,294.0062381089677,15.544264784675875,-41.51506306413057,-357.3784322842748,402.1476640928553,1544.913063094918,-1324.944521051487,-2387.170229221432,990.220233116837,926.9846492574441,525.3163304069552,-91.94675208423271,-92.33538151167275};
    const double poly_kp_0070[144] = {-0.15192360321840356,-0.15117241414930538,-0.14747293641144077,-0.1482065574279105,-0.14884418255989096,-0.14815985957758765,-0.1475741096423253,-0.14733437026856508,-0.14755022328282766,-0.14783716620606283,-0.14573136625380467,-0.14422196929824913,-0.14237902880581776,-0.14048211179922454,-0.14080461828258103,-0.14145734950040117,-0.1396902603568867,-0.13842887916421542,-0.13878116325402992,-0.13530346112364353,-0.1341058343929257,-0.13497165648194612,-0.13707157585414648,-0.1388734779878905,-0.13788197810876063,-0.14018830100078117,-0.13689455294622493,-0.13374198064602602,-0.13366338906330788,-0.1363161724945968,-0.13974252098602302,-0.14237883357099976,-0.14395388489144595,-0.14575676475866847,-0.1464650258094125,-0.14649702906136117,-0.14691005697997994,-0.14581152170825648,-0.14689935910216864,-0.14563167103205302,-0.14809183803468384,-0.1483048717456115,-0.14924586122919176,-0.15065991983023608,-0.15108969539297817,-0.14953719363274115,-0.15000511778305967,-0.1489817461327499,-0.14892274326014457,-0.1504466913952493,-0.15042909180711223,-0.150133184949315,-0.15396215838863267,-0.1531274351713909,-0.15401528662582642,-0.15347455671951624,-0.1550803582965802,-0.1540520732883485,-0.15393267970996186,-0.15294588838780634,-0.15382915239252895,-0.15331024732086085,-0.15428351235491475,-0.15260540329531272,-0.1530766381442663,-0.1549832641387291,-0.15390023794531577,-0.15241764024298365,-0.15351993105569495,-0.15244277083044544,-0.1517093948280427,-0.15111155192201725,-0.1524455210808437,-0.15214261544858235,-0.15197089831389304,-0.15071489829064008,-0.15004167196576057,-0.15012969105038737,-0.14751530152027317,-0.14811583289362878,-0.1494996893822903,-0.14987407657231794,-0.1496856997404313,-0.14913061309769524,-0.15086073600658076,-0.15236333635478916,-0.15110636802415597,-0.15204732998836681,-0.15187913661639446,-0.15176953777314453,-0.15240207175951526,-0.15444644805694707,-0.1539732426802559,-0.1544908212857668,-0.15507018676848777,-0.1563983290675426,-0.15644502105011282,-0.15459151833296175,-0.1535295780842503,-0.15386044842592403,-0.15436548392810062,-0.1552056998243963,-0.15624709653816374,-0.1547432059032801,-0.15729454919877967,-0.15801814935226996,-0.15877892400642532,-0.15738986361418178,-0.1580623228008227,-0.1613364565375257,-0.15774561938802464,-0.1607843891554939,-0.16303152081902447,-0.16002085212877468,-0.15811196877168854,-0.15971932968233907,-0.1600660193276272,-0.15878958371464674,-0.15773914612299184,-0.156649341841473,-0.15805662373829968,-0.1572043773399144,-0.15647423646918054,-0.15764905880437277,-0.15620103466167404,-0.1556899096712984,-0.1553387374830476,-0.15391513834566886,-0.1549267459100832,-0.1552238094716955,-0.15435271960320024,-0.15470656875953148,-0.15476811779093397,-0.15619115476614645,-0.1558437295024172,-0.15660679380132675,-0.15572547149315347,-0.15534104226190762,-0.15652718886412775,-0.15556383374331623,-0.1572730591660083,-0.15680794176555204,-0.15712964782677083,-0.1554081986042639};
    const double poly_kq_0070[144] = {-0.001004542981254503,-0.0008753887926847303,-0.0014240429792831903,-0.0010367168392765105,-0.0011105558332062762,-0.0010944689308746251,-0.001040022142409595,-0.001356455955217541,-0.0017089586534285545,-0.0017847908480458988,-0.0016812645696031147,-0.0016722364672109833,-0.002091100370230831,-0.002309455307941499,-0.0019116960507567981,-0.0009078033370344406,-0.0004585261770404904,-0.0005737444125194324,-0.0013321048366624984,-0.0019400594829399914,-0.0011823460787247682,-0.0013395067501515041,-0.0011618503184497816,-0.0016715485358489647,-0.0015042453800312367,-0.0015314732044162116,-0.0016591069212442506,-0.0016648270546252247,-0.0013798594300837642,-0.0014437100208848916,-0.0013264980386553658,-0.0016286264211449683,-0.0020101990330633306,-0.0013138535752834114,-0.0018285465644739666,-0.0015412994677791465,-0.001665120698643644,-0.002068065195762232,-0.0020414786654424067,-0.0022439514250340454,-0.0017512421337933218,-0.0013404808459375423,-0.001255716151414647,-0.0006142090026790109,-0.0004948035958347736,-0.001695091821766098,-0.002515760518664868,-0.0022475001703446515,-0.0021018387225147216,-0.0019302472968641443,-0.002202053878279091,-0.0012957839364157201,0.00016456228359348584,0.00026232341391589365,4.870393073813546e-05,-8.225856309023018e-05,-0.0005426009227762011,-7.200257150336054e-05,-0.0006537949709368607,-0.001319638870806556,-0.0011128044471094424,-0.001667417986583924,-0.000613971709778764,-0.0011094745790367982,-0.001173507741766047,-0.000706832305305459,-0.0006406961717024202,-0.0011876782886028384,-0.0011773483826466437,-0.0011297537638459798,-0.0012392379845606804,-0.001385978177296163,-0.0014859645999804228,-0.0012762929864265557,-0.001389224761134375,-0.0011873564605404952,-0.001409939167338151,-0.0011833946576321572,-0.0018505064374182898,-0.0017411487836192605,-0.0019275715630271333,-0.0010363323654819516,-0.0008958860680134524,-0.0005571504847871816,-0.000381403978982876,0.00016280826161387465,-0.00029662205512540887,-0.0005025782910488328,-0.0004180159723374653,-0.0006860222957318449,-0.0005640419399909486,-0.0007903256950082363,-0.0007074232493900472,-0.00044905646819685306,0.0006091742411963791,0.0009640487998324559,0.0008896934019145453,-0.00035697684274472774,-0.0003564744675088551,-0.000502380540442895,-4.8792137797623616e-05,-6.155317905849036e-05,-0.00039214756537326445,0.0001974599470610082,0.0031893944269216407,0.002941331492448047,0.003007993917219809,0.002857203476741684,0.00210650553568687,0.001792492146657436,0.0017512191870049823,0.002487681807558246,0.0021771174708736982,0.002659736475765315,0.0026581116471197227,0.002633931637145573,0.002349753327328335,0.0018882648195141225,0.0017938375494481902,0.0012764755864217094,0.0015688385893186322,0.001127609769772029,0.00201667945871694,0.001841434724532392,0.0015283607239522312,0.001415608499330577,0.0016473711881376302,0.0008722817005373392,0.0006495150867388212,0.00029622741507276433,0.0010010255351943023,0.0007091604185348459,0.0004999877580157726,0.00038395357513136617,0.00024202835206281101,0.0006307601109857045,0.0002313919123334263,0.00031902563397477986,0.0019606719101590837,0.001967000536876196,0.0014060526171364028,0.0010114269311739335,0.0010799421367603144,0.0008409750925782647};

    const double poly_alpha_0125[144] = {1.051871681905592,-0.46060726513497885,-0.42403838329517307,-3.937791233417899,3.929366669299962,14.539714138655782,-7.623737944199387,-22.17229353480822,5.802911654327116,14.870183921437533,-1.5160994074216214,-3.6094537540928844,-19.405145295651984,-2.3688760846999335,17.649443607721274,13.768792519660103,-54.53627536963766,-31.665791243618102,91.27028311641499,37.78868687052328,-66.2753456747591,-22.73417194854332,16.928105353853244,5.418416740929737,-3.638503872407762,-10.564906162069455,18.28258877858537,99.25793067021164,-73.35597490940567,-327.72739894972773,140.61874333590407,471.9724334052451,-109.62459454838942,-303.1265899124949,29.001002092144837,70.43644022150463,10.409512983790663,-8.43606760742135,-94.73428548992004,167.52150970095516,353.60493985254715,-769.485423346278,-632.0451537615455,1221.2477972577594,444.32659296339267,-785.0318635317877,-103.64208282258527,175.4003947402314,12.48307364902751,53.60361018158488,-64.68563111297638,-551.2578934258261,355.8646113977141,1987.3159925903221,-830.7207811079406,-3104.7386349741037,712.4357338756752,2144.115529639116,-199.8256225952133,-528.5735092707511,-17.741901139680948,82.19842407347753,231.6011105349307,-1183.4266154921997,-951.0275905225827,4738.506562988343,1798.2043426073185,-6751.466975150266,-1106.0179902732398,3801.817080464117,180.71961035915845,-712.9309525248761,-18.270632532652833,-107.5541220404622,59.77632330494949,1182.6366486356164,-569.9666456183993,-4525.151047101915,1679.026555220512,7390.138950376731,-1547.3863715952543,-5231.987373613361,450.757798480505,1280.6876178960338,14.866013990918333,-173.45675618571272,-305.2544681278188,2332.588624734404,1195.560584540418,-8659.548010259843,-2029.350435225514,10726.240719017753,512.7967518996034,-4655.016882768849,234.89846286383042,517.6898532417994,11.104964208444734,86.86537073933766,6.277000579250576,-1053.6038047596355,326.78222787431304,4232.395721660487,-1299.719312424154,-7098.725391629219,1245.7826940259035,5023.504012990505,-382.02985266716325,-1160.842669750147,-6.016662016347453,135.931691888348,207.7431554166098,-1800.704860269113,-663.4116180376449,6242.787438111872,657.3035473614456,-6498.543072181618,935.4786725839981,1878.6393965239306,-649.3866964313821,-73.50050506777194,-2.377410307318918,-23.94848670409128,-18.900556829587828,334.24336014111043,-40.630418061211735,-1411.9074533332553,318.1911145839937,2422.8062627793156,-328.8069592236127,-1692.815689733065,129.97643298494435,349.8879810019343,0.9960303254106151,-35.890765559565864,-57.65119383283016,489.9436031995567,140.4205234258386,-1635.622481312781,20.168517805872902,1562.615340546709,-543.4728496800177,-555.5894719206663,207.98372635845004,225.51458828708664};
    const double poly_beta_0125[144] = {-0.6075203010862941,20.728806497017835,4.413372071503389,-13.439103656738352,-15.595935085820777,29.012925264174637,22.993038557855716,-32.653589021261176,-14.781374951298925,17.461360386466566,3.4535802417981056,-3.5833044820380655,-0.6137690608731692,4.493459734148195,1.9230570220737566,-19.91223880362215,-8.533570305836438,47.657849830027295,12.314928273381303,-59.49427079452241,-7.805796787408971,35.37647985212881,1.8695938224314137,-7.871648954324682,1.5046537678909786,-17.389705582562094,-29.465304886450053,55.764676119071076,114.31083866316075,-77.29141654884852,-171.60405952236832,45.032682634453366,111.07742045574558,-5.502143626380587,-25.78744138578808,-2.405267693949696,-0.2474719416431128,-29.0826778867987,-6.1292948386489154,159.3193187778178,-11.06134282234215,-386.7944069530334,52.76556828731663,463.8505722859062,-34.88573704006177,-257.1424312330919,3.70340262256979,52.88551293342234,-4.461034290268333,42.39515572693783,89.82150694127535,-100.53654380151468,-378.4749253665667,-58.11152786266915,605.6796187468016,395.9268550780237,-415.2769851486395,-400.7026400133429,101.57352031578486,122.82135772741842,2.900822369272575,83.35487683989942,-6.310800464483137,-519.6997864550893,253.57475779710052,1521.7167450792233,-525.7952935368588,-2208.5859178373585,227.97588871592484,1456.3318458041113,11.561493759523685,-355.1820248664564,6.146212774995487,-59.34712970576743,-124.57647218424896,56.140330050951185,579.3232608685555,402.4153397612975,-1028.9973940563261,-1088.5602689018415,800.0543701172039,945.9506572091706,-224.7325771107225,-268.9944631817071,-6.1933931090013195,-111.69334612370051,23.52108365829399,749.9831346305737,-504.0124848605887,-2554.13289654363,716.5071524624742,4221.293305861605,148.87972939909668,-2994.826649492434,-268.05811763272186,756.5526852077132,-3.9033913719258226,38.680220508920186,77.15011572413786,16.15946966103166,-405.7779769385598,-415.78871100738166,813.7774996091099,905.8434062050059,-743.0981146374168,-706.4278232712749,242.54088766820846,193.08476968364215,4.923200894154739,68.32294738145512,-18.32644774418025,-469.59322502378035,349.79454626260093,1749.4075428215929,-211.44467393196612,-3023.7433500310385,-614.4983447860309,2043.2336106118541,358.1367375048923,-465.70527878547927,0.9047010690881052,-9.25536317585886,-16.807053523406275,-17.47709847083935,102.25076397486211,140.59980192374883,-227.13222951782643,-285.53631353854774,232.9797505730768,234.57742394820374,-75.60143532981859,-89.25078194263739,-1.321593806030691,-15.429834577890635,4.704039955260505,102.78219248416033,-79.62936973691714,-399.1578092499136,-42.16884451952321,681.6400378500966,282.58030867329956,-400.1280273608032,-114.6468699375254,86.59942254022529};
    const double poly_kp_0125[144] = {-0.18229192836793323,-0.18199455807681292,-0.18103360458460332,-0.17991490188428916,-0.17815116687252244,-0.17807447413401456,-0.17772788206214554,-0.1768951227385781,-0.17676444157343585,-0.1763341443508154,-0.17680720890017212,-0.17673951077469222,-0.17601275715153308,-0.17634369770325495,-0.1764555551066289,-0.1755718579613153,-0.1758595911682561,-0.17615017791366663,-0.17608553399965568,-0.17613682681798404,-0.17600857305939485,-0.17632737769793502,-0.17626277460900883,-0.17597503205670734,-0.1760810938353426,-0.17664308267530127,-0.1765416876132822,-0.17641422556895645,-0.17659301105702677,-0.1769643092268398,-0.17692754006247083,-0.17660434494494714,-0.17694870602820034,-0.1760459893523109,-0.17619981259480855,-0.17699538859356304,-0.17719916658616658,-0.17779742082227795,-0.17735320611645267,-0.17816939492535014,-0.1773759750004663,-0.17802735134661424,-0.17813155386818733,-0.17817220705855533,-0.17830897730106934,-0.17835474578311958,-0.17778745621076117,-0.17842046671337208,-0.17901754323710006,-0.18227685843809313,-0.18183578260079472,-0.18159533576728346,-0.18769391832918003,-0.18775694667172127,-0.18764775402339093,-0.18753210151954686,-0.1861594196629806,-0.1858791911164509,-0.18625901096030567,-0.18632893169278344,-0.18497627482148946,-0.18505090031702096,-0.18509385651362192,-0.18576911933154489,-0.18535545152807595,-0.18541199157367866,-0.1846733253375733,-0.18382196327256128,-0.18413894993216579,-0.18290330079033595,-0.18263345421599952,-0.18133506146812828,-0.18164540480725055,-0.18115248835792727,-0.18197224647118046,-0.1822580607748528,-0.1834060823436783,-0.18382002714149032,-0.18372050732350137,-0.18263977908119056,-0.18159751871448848,-0.17984034212953254,-0.1801952666007687,-0.17957040986358808,-0.17943092233950833,-0.18069616635824545,-0.18278193052823857,-0.1838370848302986,-0.18441631107097461,-0.18354850358171698,-0.18414970052050847,-0.1835341583124371,-0.1824935978728374,-0.1825675292474075,-0.18423545046782538,-0.18615561442285466,-0.18593398571978206,-0.18592830354532397,-0.18483490015583837,-0.1846492917831704,-0.1858565263820683,-0.18630595403064665,-0.1868028965167119,-0.18658825837801313,-0.18987378959779574,-0.19029621497831478,-0.1900825426974252,-0.19055242420076757,-0.190424619192958,-0.19120731449846445,-0.1923857262433134,-0.1926186653191782,-0.19233262685210364,-0.1909564966139351,-0.1897300355781942,-0.1897637342648008,-0.19003080907416786,-0.19050068302183173,-0.19061270674115346,-0.19136620953634212,-0.1899670286718873,-0.18896509328811625,-0.1882893754439248,-0.1866749033335598,-0.18792545121389234,-0.18810938347740885,-0.18738796061976448,-0.18736390145579818,-0.1890279243623145,-0.18884078861766412,-0.18862555647199772,-0.18923828962356823,-0.18938984190843736,-0.18850948681404617,-0.1892089918362658,-0.1898898628774961,-0.18916467504985543,-0.19078094493960576,-0.18928734600589645,-0.18797016986399917,-0.18873338151871943,-0.19006813299843686,-0.19040019772319228,-0.19050117311907377};
    const double poly_kq_0125[144] = {-0.0004936106870932184,-0.0003798389336867092,-0.0005327898313211222,-0.0004440706164779305,-0.00038400007402029,-0.0004460799853080376,-0.00047692728964842795,-0.000333827441739723,-0.000407985701953007,-0.0005281522931785853,-0.0004218894155235676,-0.0004092546372755079,-0.00041221825722003,-0.00020050206801944197,2.7064180192040146e-05,-2.242504428963548e-05,-0.0004508164475328918,-0.0005735285649251479,-0.0005445266562810811,-0.0003593060062488779,-0.0003995917517138071,-0.0004287454926178216,-0.000443199241067603,-0.0005210364599675801,-0.0004773229070155072,-0.000596001641714655,-0.0004660100536064874,-0.0005221441044307317,-0.0005371056958295119,-0.00039056389886484624,-0.00032304628005104194,-0.00042939590185734136,-0.0005323105532300956,-0.0005538187035455648,-0.0006292040975035236,-0.00020674091530338359,-0.00012389316459894214,-0.0003008982664078524,-0.0006586381612400248,-0.0006602402764361143,-0.0005280287544105684,-0.0006107331132916633,-0.0005130100983370497,-0.0005249779116841113,-0.0006684432711764431,-0.0006043961133446882,-0.0007368125028157683,-0.0007736924491470333,-0.0006973632494925329,-0.0006997413648303926,-0.0006944531229729434,-0.0006356810717058096,0.0005957109579670534,0.0005923488781468396,0.0005233806851742868,0.0006791917157071453,0.0006282434654899132,0.0004481818601863493,0.00041576832157037987,0.0004965558654170724,0.0005554299439904148,0.0005100738623251409,0.00039540480918466385,0.0002600311651558349,0.00024498774964542896,0.0004355643715349623,0.0004550978199567192,0.00042251396970309133,0.0007966532835106782,0.000784906245161986,0.0007244832976998266,0.0005444243292711567,0.0005173292740907587,0.0007417475743206138,0.0006300328950625122,0.0007113414895322649,0.0006015814487443585,0.0005918126657988266,0.0004473956271279092,0.000337975676166523,0.0003977904478505028,0.0006604399194926585,0.0007491649770798347,0.0007743638380602213,0.0006562738581274249,0.0007638552058040274,0.0007980690433884885,0.0008491115302178519,0.0008346500324048116,0.0006473625569801698,0.0005307328959746794,0.0004960752769122317,0.0005074968759451705,0.000571001617189483,0.0005745255820235162,0.0006397391157773804,0.0007823093371201172,0.0006299058587230179,0.0003224782957334185,0.0003712104715188396,0.0003774852715405702,0.0004498257213191235,0.00047781079691140113,0.00027008154385275084,0.0035880090454048813,0.0036110839443964085,0.00399531939429316,0.003985013972036513,0.003944336153315161,0.0037925545112235393,0.0037621148481890593,0.0035240546578922307,0.003480195502833261,0.0033645250428193966,0.003273464406852562,0.003136526880957114,0.003328667979445065,0.003214356790516399,0.0031770522873112417,0.0030976666187424065,0.002893933518590702,0.002950096833834498,0.0029455194890289013,0.0029844344833976546,0.0029578805229485547,0.003079234091691303,0.0031346367888543925,0.0030644365711337868,0.003175695876876845,0.0032486044414388664,0.0032450427096964865,0.003336904440463954,0.0034637254187856984,0.003233800487276569,0.0032116464155968094,0.0033310990595739123,0.003484022333991685,0.00366921021661666,0.0035666994892932955,0.003474142682350101,0.0035315272149504997,0.0037785540885491597,0.003859941159533115,0.003882573831941419};

    const double poly_alpha_0175[144] = {0.8975804636022932,-0.5071170716154706,0.6051299349088602,-2.854808956376249,1.4092057776999607,9.222994095501365,-4.542555823679108,-12.801173061483455,4.087028100280175,8.102756535649242,-1.1808229130990378,-1.9093075470071712,-17.60465504864161,0.18619412908983146,3.196419354077947,-5.729606270155167,-4.184422906071654,30.045460232899107,17.5383239149458,-55.94003299082972,-19.962628781812924,43.59873671521701,6.675456163998784,-12.088546627575468,-1.1654959053945837,-3.48932247696904,-5.830793533422706,40.132033313114036,62.49069582380745,-157.2717595300241,-151.31049592537846,254.95789402659165,136.44321860416807,-182.21403643132678,-41.20125690862966,46.9332591048856,2.943612667728271,-15.52934109712082,0.7046744015767814,211.316767598711,-4.4790903055606215,-913.5168691390991,-93.2398832121377,1569.8316795319554,130.03639427841205,-1168.58733365918,-46.195445585450116,313.8577951163106,2.2168807482179176,-9.062938987796725,27.177960842737725,-18.655166391200176,-417.41946343164165,438.14275252145706,1138.5680355599973,-1107.7208653495888,-1067.0795016869665,1001.1080026169263,321.3708071937222,-295.4356665681251,-3.6132610811845454,58.16447047229261,-56.13954271735184,-902.5378166703822,203.1779879221052,3820.1798349126693,35.607112795045396,-6334.891091351322,-227.78360012556823,4557.985889844409,94.58760796275833,-1184.255942094637,-1.7789560638671276,32.84877414377816,-65.14575815701482,-73.36161825786066,1017.7134270368673,-870.2568243407608,-2784.9743371823024,2727.675063024589,2491.9419726910055,-2584.104816349458,-683.5966185068824,740.5125163056478,1.392723908944106,-76.72643356174486,126.65510091891142,1298.9374001339293,-549.6955722557497,-5143.657814518787,388.5729367749059,7667.829049954777,20.04325993968987,-4956.12962214394,-75.747821837051,1165.7145023573376,0.6607239637349664,-28.013952568703274,60.82333758873813,69.69047952420397,-909.2618244530966,903.9021055216617,2336.649559161634,-2760.749065831533,-1770.3556928937064,2394.306036891071,335.01983081605334,-547.9345947218823,0.6026547991542941,42.7257699623321,-96.82858678750591,-755.7491908322928,459.40755646027804,2622.244732047153,-337.8632470428522,-3161.7035423846373,-141.48343520132937,1718.5480719067361,184.81820313690315,-402.9732049223394,-0.07862903941347099,7.481157747600292,-18.43325476790831,-16.56886593295566,260.2055948076798,-304.32730840923085,-592.1618973698853,822.4843462734827,299.06446331298207,-529.7830235771293,14.29718301590339,9.389384895283243,-0.3802004248065671,-8.613913180705062,24.292601694186835,153.94824404097298,-118.63282818445366,-431.8925191353724,41.53247333928751,362.5136606926421,159.5455570693663,-238.9209875368519,-118.34126141276721,128.38924296481875};
    const double poly_beta_0175[144] = {-0.49645621699459913,18.55589668142967,1.5669071968861568,0.7807279658574554,-3.038358150577285,-10.225612841279418,1.1352056745023993,20.36191571401031,1.3103605544109067,-16.118091024765942,-0.734625123834368,4.4456444080893105,-0.6383896063923347,-1.122486076834646,3.1462303630011608,27.381096562633765,-13.939271890378567,-98.97743462503198,19.381016593503723,142.55158522964703,-10.611066318373764,-90.9199145056392,1.9519381709351402,21.447044875005396,0.6163410146387247,0.18442078859800448,1.7977517904417648,-91.15727369235444,-51.27373036064138,306.1371401502281,152.2168059934526,-387.3511118496934,-147.52663527230774,216.83778962703263,45.521971141100536,-45.746018897898324,0.09866665654671891,19.475131700139286,-27.156315310332772,-295.3577344870667,96.01856003830596,1248.5651858087656,-94.79457323240428,-2103.8635041775315,11.677593464523905,1525.898101162372,11.72163800644696,-397.8962103099723,-2.2273357321235494,-33.97096864486606,-40.15756029921398,579.4113419985777,376.6664770008007,-1981.6193868431792,-974.8570126475613,2773.7592141602645,886.9615462554013,-1787.8940688696193,-259.6462801088704,444.9510750063319,1.5663206060286559,-55.82553264846559,66.63029435488012,801.2198353110524,-130.2041820293364,-3709.333183041017,-151.74578009266185,6613.116567440901,417.6316142329844,-4884.771424858527,-186.89946742787507,1255.8504819131415,3.770636666892088,68.00215084485936,78.21236857284887,-1127.5662229894306,-549.0378000173187,3938.675804432616,1225.2593581022352,-5753.660824596781,-867.7654146928683,3947.966495995781,164.94639661031934,-1054.0059902835303,-4.369281493296242,62.04898233970872,-65.13380986879062,-806.4483799335796,-89.05882307075129,4024.8364035361,889.4712328975294,-7213.848706961493,-1279.2160184021106,4953.206198071258,491.4015723578735,-1078.5642864086278,-2.9765404143783627,-50.20177431615453,-49.717231206575065,859.2714062938304,209.08907549169024,-2981.567850292837,-218.2517969299013,4357.014151287484,-242.6377665614156,-3047.3585744049124,206.2159074934499,836.7592545188916,3.9159385911328,-28.88626349041519,25.541812821208694,304.3714965641641,240.33401376481436,-1712.3585385801682,-1049.440134852531,2961.9303867958365,1351.3218307415639,-1580.808611103768,-481.25364233621735,128.658934758592,0.8579309482937418,12.688838630843378,9.804009575360638,-224.65762282409509,3.7576679246456486,756.4482168956466,-127.6444387231375,-1078.9084049017395,267.91544838364905,761.8565454363836,-93.61021937506959,-218.02675251720194,-1.1372165058677695,4.544490968724141,-2.937009562738524,-27.159211811721775,-98.12241792669334,223.6850699114771,349.35161484994234,-346.559015753496,-410.9642256561849,25.48464692348052,108.63038211953743,90.35490942247256};
    const double poly_kp_0175[144] = {-0.18963840897511666,-0.18942806965066375,-0.18970085712487939,-0.1902339209831032,-0.18979889392723112,-0.18960629778261534,-0.18955978808551308,-0.189899833620236,-0.19041751851189267,-0.19018656027818598,-0.19044691106567516,-0.190291816162055,-0.19032714116426197,-0.19002330753574306,-0.18969869155988844,-0.19022989278388752,-0.1902797769946406,-0.1900427357990359,-0.19000905643256874,-0.1896486373379731,-0.18994168666144323,-0.18982497303243162,-0.19006028258842408,-0.1900237777972183,-0.18954519181888066,-0.18976183829255722,-0.190335287768239,-0.1896960670641708,-0.19002535349478858,-0.18998082553715753,-0.18998828413343763,-0.1901634860003464,-0.1899506536076454,-0.19000748877529397,-0.18945611230793394,-0.189399539657454,-0.1891221840145481,-0.18922769063846878,-0.18920798752682974,-0.18911513990431697,-0.18857251588458193,-0.18944716104787365,-0.1895066799746535,-0.18898029293399474,-0.18918173779822564,-0.18926159327545367,-0.1889803151276673,-0.18890203108575931,-0.1891134797169016,-0.18909291395877342,-0.1896578592938819,-0.18941938093295638,-0.19643971732623572,-0.1957396900231651,-0.19578707663889716,-0.19386494556856756,-0.19253320388643105,-0.19249809093956358,-0.19278106226359334,-0.1918304026562388,-0.19146561449633925,-0.19209490957260408,-0.1915903617057838,-0.19140915677369993,-0.19249153375143843,-0.19209511772807525,-0.19293954835420107,-0.19206961960512006,-0.19098560110581694,-0.19082494517825485,-0.19084492021823152,-0.19049920760082406,-0.19156880950409788,-0.19226059549921007,-0.19199471061916318,-0.19215990918125533,-0.1919582184166893,-0.1917703705738258,-0.19173430136715863,-0.1922494812171522,-0.19136669525960756,-0.19120946392009422,-0.19151889453123244,-0.1910964161623942,-0.1916500161843544,-0.1912011725773374,-0.19194067628477085,-0.1946084527326228,-0.19447453520203592,-0.19394765323581384,-0.19257495678251207,-0.19146351767316788,-0.19079011743936444,-0.19128606717585947,-0.19093104506769026,-0.19165633559119583,-0.19192635867312247,-0.19229404501867037,-0.19168128065574844,-0.19106965550932326,-0.19167442516930003,-0.19420861870791722,-0.19556806425675535,-0.1953926423556571,-0.19921422685840914,-0.19894373344848856,-0.19900563476226696,-0.19557305555358057,-0.19808651325515386,-0.19951383489457394,-0.19956789553262075,-0.19962318339658705,-0.1989868177043655,-0.19770065002126636,-0.19731332165699003,-0.19802021229009267,-0.20069745883526133,-0.20064196790394037,-0.20044360968946529,-0.20040122331568336,-0.2014478429614364,-0.19979270418574757,-0.1965126848138109,-0.1945824234879555,-0.19394009976712098,-0.19483685596320327,-0.19556372185163196,-0.19681653855944664,-0.19836314825608456,-0.1983851813992958,-0.19920998367498574,-0.19849096472139005,-0.19584065526540412,-0.195076034629175,-0.19635957402176912,-0.19698181427333572,-0.19691043316965423,-0.1962963561910535,-0.19642472818901863,-0.1954898971523328,-0.19683053867369704,-0.19928394848585104,-0.19979699213129545,-0.1994571479583999};
    const double poly_kq_0175[144] = {-0.0001292374227856979,-5.2626701199877094e-05,-2.0872253400574134e-05,-0.00013106495592815623,-5.0643295493434696e-05,-2.6275214908638474e-05,3.094771577351491e-05,-3.6754045219075636e-05,-6.189084061250694e-05,-5.2229604436097674e-05,-5.334070826724089e-05,-0.00015801935679364727,8.121173317836261e-06,1.8946072519758202e-05,-2.7057845802140884e-06,-4.5651452512339944e-05,2.7053438076227786e-05,3.868280784309103e-05,2.2414042632210075e-05,-5.261411386915026e-05,-3.017958303018374e-05,-5.418671736597434e-05,-1.548387296793535e-05,5.497309028774409e-05,-0.00010329691246559367,-0.00015157221372107796,-0.0001649889561973068,-9.569304769398539e-05,-0.00015722497150603385,-6.463084659844133e-05,-7.591734344137384e-05,-4.142782349011307e-05,-0.00011885799151121587,-0.00012183159296014051,-0.00016461872898449877,-0.0001668552983217851,-0.0001511938498988197,-0.00026220913711769286,-0.00032140546850757635,-0.0002893798373455492,-7.014387826772681e-05,-8.781331122029101e-05,-1.4340219576345037e-05,-5.540617521161414e-05,-4.840264495390945e-05,-0.00015364686840764084,-0.0001894694873714516,-0.0001428883149703276,-4.961738887880379e-05,-6.498852497742039e-05,2.4763994776850032e-05,-8.282739438922315e-05,0.0010836987209337347,0.0009977786946956326,0.0011490565803834864,0.0011090108878370117,0.0011445504838054745,0.0010818009113825267,0.0010872736003111265,0.0010425222198976345,0.0009782543267098168,0.0010031623703016506,0.0010723861632269249,0.0009504709805224479,0.0010359024114675616,0.0010758693762528055,0.001023727371130402,0.0010926878703282686,0.0010469026167818987,0.0010575211234581721,0.0009953538498493743,0.0009570132180405617,0.0009360320577426939,0.001007958828190242,0.0010007775420882133,0.001000788464778093,0.0009373998595070451,0.0009884065079902825,0.0009268078791098655,0.0009964509435066921,0.00102232625180602,0.0010579088726169823,0.0010823476469633301,0.0011802628739114958,0.0012431756934751432,0.0012123559503409336,0.001157055792874143,0.0010460207594733073,0.001045500219794116,0.0010217494121853173,0.0010242854398901922,0.0010763040254630441,0.0011039973451283014,0.0010468723525261922,0.001167701583751593,0.0011103382121584746,0.0012170772166354247,0.001152202477488813,0.0011895054881579151,0.0012013377812399942,0.0011980916941189367,0.0011724525930419413,0.0012546576356350426,0.0010627891529215825,0.0044977319129694705,0.004500826835108011,0.004502133060021819,0.004486205026617459,0.004294773425923961,0.004177349562297617,0.0040954635837230574,0.003835815733882864,0.0038391902629072564,0.0037775590790333096,0.003726237209613551,0.00371438268331918,0.0037765788792175022,0.003897024090693555,0.0038239403665629524,0.0037738442310869262,0.0038261094023929266,0.0038487249471844537,0.0037328580940081736,0.003683889833153631,0.003654262128208568,0.0036447004764808034,0.0036349211371071357,0.003554395394545203,0.00348644213748464,0.0035102047402361357,0.0036261834758742886,0.0038138825324162155,0.00394648483909206,0.004058509001158853,0.004146994687466823,0.004138740578278928,0.004239044250608032,0.004312942392717773,0.004302085754166763,0.004354328711567063,0.004352737301991502,0.004382553997789149,0.004311087899931734,0.00439589173009285};

    MAV_AUTOPILOT       _firmwareType;
    MAV_TYPE            _vehicleType;
    FirmwarePlugin*     _firmwarePlugin;
    QObject*            _firmwarePluginInstanceData;
    AutoPilotPlugin*    _autopilotPlugin;
    MAVLinkProtocol*    _mavlink;
    bool                _soloFirmware;
    QGCToolbox*         _toolbox;
    SettingsManager*    _settingsManager;

    QList<LinkInterface*> _links;

    JoystickMode_t  _joystickMode;
    bool            _joystickEnabled;

    UAS* _uas;

    QGeoCoordinate  _coordinate;
    QGeoCoordinate  _homePosition;

    UASInterface*   _mav;
    int             _currentMessageCount;
    int             _messageCount;
    int             _currentErrorCount;
    int             _currentWarningCount;
    int             _currentNormalCount;
    MessageType_t   _currentMessageType;
    QString         _latestError;
    int             _updateCount;
    QString         _formatedMessage;
    int             _rcRSSI;
    double          _rcRSSIstore;
    bool            _autoDisconnect;    ///< true: Automatically disconnect vehicle when last connection goes away or lost heartbeat
    bool            _flying;
    bool            _landing;
    bool            _vtolInFwdFlight;
    uint32_t        _onboardControlSensorsPresent;
    uint32_t        _onboardControlSensorsEnabled;
    uint32_t        _onboardControlSensorsHealth;
    uint32_t        _onboardControlSensorsUnhealthy;
    bool            _gpsRawIntMessageAvailable;
    bool            _globalPositionIntMessageAvailable;
    double          _defaultCruiseSpeed;
    double          _defaultHoverSpeed;
    int             _telemetryRRSSI;
    int             _telemetryLRSSI;
    uint32_t        _telemetryRXErrors;
    uint32_t        _telemetryFixed;
    uint32_t        _telemetryTXBuffer;
    int             _telemetryLNoise;
    int             _telemetryRNoise;
    unsigned        _maxProtoVersion;
    bool            _vehicleCapabilitiesKnown;
    uint64_t        _capabilityBits;
    bool            _highLatencyLink;
    bool            _receivingAttitudeQuaternion;

    QGCCameraManager* _cameras;

    typedef struct {
        int     component;
        MAV_CMD command;
        float   rgParam[7];
        bool    showError;
    } MavCommandQueueEntry_t;

    QList<MavCommandQueueEntry_t>   _mavCommandQueue;
    QTimer                          _mavCommandAckTimer;
    int                             _mavCommandRetryCount;
    static const int                _mavCommandMaxRetryCount = 3;
    static const int                _mavCommandAckTimeoutMSecs = 3000;
    static const int                _mavCommandAckTimeoutMSecsHighLatency = 120000;

    QString             _prearmError;
    QTimer              _prearmErrorTimer;
    static const int    _prearmErrorTimeoutMSecs = 35 * 1000;   ///< Take away prearm error after 35 seconds

    // Lost connection handling
    bool                _connectionLost;
    bool                _connectionLostEnabled;

    bool                _initialPlanRequestComplete;

    MissionManager*     _missionManager;
    bool                _missionManagerInitialRequestSent;

    GeoFenceManager*    _geoFenceManager;
    bool                _geoFenceManagerInitialRequestSent;

    RallyPointManager*  _rallyPointManager;
    bool                _rallyPointManagerInitialRequestSent;

    ParameterManager*    _parameterManager;

    bool    _armed;         ///< true: vehicle is armed
    uint8_t _base_mode;     ///< base_mode from HEARTBEAT
    uint32_t _custom_mode;  ///< custom_mode from HEARTBEAT

    /// Used to store a message being sent by sendMessageMultiple
    typedef struct {
        mavlink_message_t   message;    ///< Message to send multiple times
        int                 retryCount; ///< Number of retries left
    } SendMessageMultipleInfo_t;

    QList<SendMessageMultipleInfo_t> _sendMessageMultipleList;    ///< List of messages being sent multiple times

    static const int _sendMessageMultipleRetries = 5;
    static const int _sendMessageMultipleIntraMessageDelay = 500;

    QTimer  _sendMultipleTimer;
    int     _nextSendMessageMultipleIndex;

    QTime               _flightTimer;
    QTimer              _mapTrajectoryTimer;
    QmlObjectListModel  _mapTrajectoryList;
    QGeoCoordinate      _mapTrajectoryLastCoordinate;
    bool                _mapTrajectoryHaveFirstCoordinate;
    static const int    _mapTrajectoryMsecsBetweenPoints = 1000;

    QmlObjectListModel  _cameraTriggerPoints;

    QmlObjectListModel              _adsbVehicles;
    QMap<uint32_t, ADSBVehicle*>    _adsbICAOMap;

    // Toolbox references
    FirmwarePluginManager*      _firmwarePluginManager;
    JoystickManager*            _joystickManager;

    int                         _flowImageIndex;

    bool _allLinksInactiveSent; ///< true: allLinkInactive signal already sent one time

    uint                _messagesReceived;
    uint                _messagesSent;
    uint                _messagesLost;
    uint8_t             _messageSeq;
    uint8_t             _compID;
    bool                _heardFrom;

    float                _ffMean=0;
    float                _ddMean=0;
    float                _wMean=0;
    float                _windAvgCount=0;

    int _firmwareMajorVersion;
    int _firmwareMinorVersion;
    int _firmwarePatchVersion;
    int _firmwareCustomMajorVersion;
    int _firmwareCustomMinorVersion;
    int _firmwareCustomPatchVersion;
    FIRMWARE_VERSION_TYPE _firmwareVersionType;

    QString _gitHash;
    quint64 _uid;

    int _lastAnnouncedLowBatteryPercent;

    SharedLinkInterfacePointer _priorityLink;  // We always keep a reference to the priority link to manage shutdown ordering
    bool _priorityLinkCommanded;

    // FactGroup facts

    Fact _rollFact;
    Fact _pitchFact;
    Fact _headingFact;
    Fact _rollRateFact;
    Fact _pitchRateFact;
    Fact _yawRateFact;
    Fact _groundSpeedFact;
    Fact _airSpeedFact;
    Fact _climbRateFact;
    Fact _vnsFact;
    Fact _vewFact;
    Fact _altitudeRelativeFact;
    Fact _altitudeAMSLFact;
    Fact _flightDistanceFact;
    Fact _flightTimeFact;
    Fact _distanceToHomeFact;
    Fact _hobbsFact;
    Fact _tempFact;
    Fact _tpotFact;
    Fact _humFact;
    Fact _pt100Fact;
    Fact _aoaFact;
    Fact _aosFact;
    Fact _windSpeedFact;
    Fact _windDirFact;
    Fact _windVertFact;

    VehicleGPSFactGroup             _gpsFactGroup;
    VehicleBatteryFactGroup         _battery1FactGroup;
    VehicleBatteryFactGroup         _battery2FactGroup;
    VehicleWindFactGroup            _windFactGroup;
    VehicleVibrationFactGroup       _vibrationFactGroup;
    VehicleTemperatureFactGroup     _temperatureFactGroup;
    VehicleClockFactGroup           _clockFactGroup;
    VehicleSetpointFactGroup        _setpointFactGroup;
    VehicleDistanceSensorFactGroup  _distanceSensorFactGroup;
    VehicleMeteoFactGroup           _meteoFactGroup;
    VehicleInsFactGroup             _insFactGroup;
    VehicleMhpFactGroup             _mhpFactGroup;

    static const char* _rollFactName;
    static const char* _pitchFactName;
    static const char* _headingFactName;
    static const char* _rollRateFactName;
    static const char* _pitchRateFactName;
    static const char* _yawRateFactName;
    static const char* _groundSpeedFactName;
    static const char* _airSpeedFactName;
    static const char* _climbRateFactName;
    static const char* _vnsFactName;
    static const char* _vewFactName;
    static const char* _altitudeRelativeFactName;
    static const char* _altitudeAMSLFactName;
    static const char* _flightDistanceFactName;
    static const char* _flightTimeFactName;
    static const char* _distanceToHomeFactName;
    static const char* _hobbsFactName;
    static const char* _tempFactName;
    static const char* _tpotFactName;
    static const char* _humFactName;
    static const char* _pt100FactName;
    static const char* _aosFactName;
    static const char* _aoaFactName;
    static const char* _windSpeedFactName;
    static const char* _windDirFactName;
    static const char* _windVertFactName;

    static const char* _gpsFactGroupName;
    static const char* _battery1FactGroupName;
    static const char* _battery2FactGroupName;
    static const char* _windFactGroupName;
    static const char* _vibrationFactGroupName;
    static const char* _temperatureFactGroupName;
    static const char* _clockFactGroupName;
    static const char* _distanceSensorFactGroupName;
    static const char* _meteoFactGroupName;
    static const char* _insFactGroupName;
    static const char* _mhpFactGroupName;

    static const int _vehicleUIUpdateRateMSecs = 100;
    static const int _windAvgNo = 5;

    // Settings keys
    static const char* _settingsGroup;
    static const char* _joystickModeSettingsKey;
    static const char* _joystickEnabledSettingsKey;

};
