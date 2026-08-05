import {App, Button, Dropdown, Space, Tooltip} from "antd";
import type {MenuProps} from "antd";
import type {MenuItemType} from "antd/es/menu/interface";
import {
    GlobalOutlined,
    EllipsisOutlined,
    EditOutlined,
    DatabaseOutlined,
    DownloadOutlined,
    ControlOutlined,
    PlayCircleOutlined,
    HomeOutlined,
    WarningOutlined,
    ScissorOutlined,
    AimOutlined,
    ForwardOutlined,
    CaretRightOutlined,
    PauseOutlined,
    ThunderboltOutlined,
    CheckOutlined,
    CloseOutlined,
    ImportOutlined,
} from "@ant-design/icons";
import type {MenuInfo} from "rc-menu/lib/interface";
import {useTranslation} from "react-i18next";
import AsyncButton from "../../../components/AsyncButton.tsx";
import AsyncDropDownButton from "../../../components/AsyncDropDownButton.tsx";
import type {Feature} from "geojson";

interface MowingAreaItem extends MenuItemType {
    feat: Feature;
}

interface MapToolbarProps {
    manualMode: boolean;
    useSatellite: boolean;
    mowingAreas: MowingAreaItem[];
    stateName?: string;
    emergency?: boolean;
    onEditMap: () => void;
    onToggleSatellite: () => void;
    onManualMode: () => Promise<void>;
    onStopManualMode: () => Promise<void>;
    onBackupMap: () => void;
    onRestoreMap: () => void;
    onDownloadGeoJSON: () => void;
    onImportOpenMower: () => void;
    onMowArea: (key: string) => Promise<void>;
    pitched?: boolean;
    onTogglePitch?: () => void;
    onStart?: () => Promise<void>;
    onHome?: () => Promise<void>;
    onEmergencyOn?: () => Promise<void>;
    onEmergencyOff?: () => Promise<void>;
    onAreaRecording?: () => Promise<void>;
    onMowNextArea?: () => Promise<void>;
    onContinueOrPause?: () => Promise<void>;
    onBladeForward?: () => Promise<void>;
    onBladeBackward?: () => Promise<void>;
    onBladeOff?: () => Promise<void>;
    onRecordFinish?: () => Promise<void>;
    onRecordCancel?: () => Promise<void>;
}

export const MapToolbar = ({
    manualMode, useSatellite, mowingAreas, stateName, emergency,
    onEditMap, onToggleSatellite,
    onManualMode, onStopManualMode,
    onBackupMap, onRestoreMap, onDownloadGeoJSON, onImportOpenMower,
    onMowArea, pitched, onTogglePitch,
    onStart, onHome, onEmergencyOn, onEmergencyOff,
    onAreaRecording, onMowNextArea, onContinueOrPause,
    onBladeForward, onBladeBackward, onBladeOff,
    onRecordFinish, onRecordCancel,
}: MapToolbarProps) => {
    const {notification} = App.useApp();
    const {t} = useTranslation();
    const isIdle = stateName === "IDLE" || stateName === "IDLE_DOCKED";
    const isRecording = stateName === "RECORDING";
    const enableTooltips =
        typeof window !== "undefined" &&
        window.matchMedia?.("(hover: hover) and (pointer: fine)").matches;
    const tooltipTitle = (tooltipKey: string) => enableTooltips ? t(tooltipKey) : undefined;

    const safeCall = (fn?: () => Promise<void>) => {
        fn?.().catch((e: Error) => {
            console.error(e);
            notification.error({
                message: t("mapToolbar.actionFailed"),
                description: e.message,
            });
        });
    };
    const menuLabel = (labelKey: string, tooltipKey: string) => (
        <Tooltip title={tooltipTitle(tooltipKey)} placement="left">
            <span>{t(labelKey)}</span>
        </Tooltip>
    );

    const moreMenuItems: MenuProps["items"] = [
        {
            key: "satellite",
            icon: <GlobalOutlined />,
            label: (
                <Tooltip title={tooltipTitle("mapToolbar.satelliteTooltip")} placement="left">
                    <span>{useSatellite ? t("mapToolbar.darkMap") : t("mapToolbar.satellite")}</span>
                </Tooltip>
            ),
        },
        ...(onTogglePitch
            ? [{
                key: "pitch",
                icon: <GlobalOutlined />,
                label: (
                    <Tooltip title={tooltipTitle("mapToolbar.tilt3dViewTooltip")} placement="left">
                        <span>{pitched ? t("mapToolbar.flattenMap") : t("mapToolbar.tilt3dView")}</span>
                    </Tooltip>
                ),
            } satisfies NonNullable<MenuProps["items"]>[number]]
            : []),
        {type: "divider"},
        {
            key: "areaRecording",
            icon: <AimOutlined />,
            label: menuLabel("mapToolbar.areaRecording", "mapToolbar.areaRecordingTooltip"),
        },
        {
            key: "mowNext",
            icon: <ForwardOutlined />,
            label: menuLabel("mapToolbar.mowNextArea", "mapToolbar.mowNextAreaTooltip"),
        },
        {
            key: "continueOrPause",
            icon: isIdle ? <CaretRightOutlined /> : <PauseOutlined />,
            label: menuLabel(
                isIdle ? "mapToolbar.continue" : "mapToolbar.pause",
                isIdle ? "mapToolbar.continueTooltip" : "mapToolbar.pauseTooltip",
            ),
        },
        {type: "divider"},
        ...(manualMode
            ? [{
                key: "stopManual",
                icon: <HomeOutlined />,
                label: menuLabel("mapToolbar.stopManualMowing", "mapToolbar.stopManualMowingTooltip"),
                danger: true,
            } satisfies NonNullable<MenuProps["items"]>[number]]
            : [{
                key: "manual",
                icon: <ControlOutlined />,
                label: menuLabel("mapToolbar.manualMowing", "mapToolbar.manualMowingTooltip"),
            } satisfies NonNullable<MenuProps["items"]>[number]]
        ),
        {type: "divider"},
        {
            key: "bladeForward",
            icon: <ThunderboltOutlined />,
            label: menuLabel("mapToolbar.bladeForward", "mapToolbar.bladeForwardTooltip"),
        },
        {
            key: "bladeBackward",
            icon: <ThunderboltOutlined />,
            label: menuLabel("mapToolbar.bladeBackward", "mapToolbar.bladeBackwardTooltip"),
        },
        {
            key: "bladeOff",
            icon: <ThunderboltOutlined />,
            label: menuLabel("mapToolbar.bladeOff", "mapToolbar.bladeOffTooltip"),
            danger: true,
        },
        {type: "divider"},
        {
            key: "backup",
            icon: <DatabaseOutlined />,
            label: menuLabel("mapToolbar.backupMap", "mapToolbar.backupMapTooltip"),
        },
        {
            key: "restore",
            icon: <DatabaseOutlined />,
            label: menuLabel("mapToolbar.restoreMap", "mapToolbar.restoreMapTooltip"),
        },
        {
            key: "importOpenMower",
            icon: <ImportOutlined />,
            label: menuLabel("mapToolbar.importFromOpenMower", "mapToolbar.importFromOpenMowerTooltip"),
        },
        {type: "divider"},
        {
            key: "download",
            icon: <DownloadOutlined />,
            label: menuLabel("mapToolbar.downloadGeojson", "mapToolbar.downloadGeojsonTooltip"),
        },
    ];

    const handleMoreClick: MenuProps["onClick"] = ({key}: MenuInfo) => {
        switch (key) {
            case "satellite": onToggleSatellite(); break;
            case "pitch": onTogglePitch?.(); break;
            case "manual": safeCall(() => onManualMode()); break;
            case "stopManual": safeCall(() => onStopManualMode()); break;
            case "areaRecording": safeCall(onAreaRecording); break;
            case "mowNext": safeCall(onMowNextArea); break;
            case "continueOrPause": safeCall(onContinueOrPause); break;
            case "bladeForward": safeCall(onBladeForward); break;
            case "bladeBackward": safeCall(onBladeBackward); break;
            case "bladeOff": safeCall(onBladeOff); break;
            case "backup": onBackupMap(); break;
            case "restore": onRestoreMap(); break;
            case "importOpenMower": onImportOpenMower(); break;
            case "download": onDownloadGeoJSON(); break;
        }
    };

    return (
        <Space size="small" wrap>
            <Tooltip title={tooltipTitle("mapToolbar.editMapTooltip")}>
                <Button
                    type="primary"
                    icon={<EditOutlined />}
                    onClick={onEditMap}
                >
                    {t("mapToolbar.editMap")}
                </Button>
            </Tooltip>

            {isRecording ? (
                <>
                    <Tooltip title={tooltipTitle("mapToolbar.finishRecordingTooltip")}>
                        <AsyncButton
                            type="primary"
                            icon={<CheckOutlined />}
                            onAsyncClick={onRecordFinish!}
                        >
                            {t("mapToolbar.finishRecording")}
                        </AsyncButton>
                    </Tooltip>
                    <Tooltip title={tooltipTitle("mapToolbar.cancelRecordingTooltip")}>
                        <AsyncButton
                            danger
                            icon={<CloseOutlined />}
                            onAsyncClick={onRecordCancel!}
                        >
                            {t("mapToolbar.cancelRecording")}
                        </AsyncButton>
                    </Tooltip>
                </>
            ) : (
                <>
                    {isIdle && (
                        <Tooltip title={tooltipTitle("mapToolbar.startTooltip")}>
                            <AsyncButton
                                type="primary"
                                icon={<PlayCircleOutlined />}
                                onAsyncClick={onStart!}
                            >
                                {t("mapToolbar.start")}
                            </AsyncButton>
                        </Tooltip>
                    )}
                    {/* Home (return-to-dock) is always available outside recording
                        so the robot can be sent back even while idle off-dock. */}
                    <Tooltip title={tooltipTitle("mapToolbar.homeTooltip")}>
                        <AsyncButton
                            type={isIdle ? "default" : "primary"}
                            icon={<HomeOutlined />}
                            onAsyncClick={onHome!}
                        >
                            {t("mapToolbar.home")}
                        </AsyncButton>
                    </Tooltip>
                </>
            )}

            {!emergency ? (
                <Tooltip title={tooltipTitle("mapToolbar.emergencyOnTooltip")}>
                    <AsyncButton
                        danger
                        icon={<WarningOutlined />}
                        onAsyncClick={onEmergencyOn!}
                    >
                        {t("mapToolbar.emergencyOn")}
                    </AsyncButton>
                </Tooltip>
            ) : (
                <Tooltip title={tooltipTitle("mapToolbar.emergencyOffTooltip")}>
                    <AsyncButton
                        danger
                        icon={<WarningOutlined />}
                        onAsyncClick={onEmergencyOff!}
                    >
                        {t("mapToolbar.emergencyOff")}
                    </AsyncButton>
                </Tooltip>
            )}

            <Tooltip title={tooltipTitle("mapToolbar.mowAreaTooltip")}>
                <AsyncDropDownButton
                    icon={<ScissorOutlined />}
                    menu={{
                        items: mowingAreas,
                        onAsyncClick: (e: MenuInfo) => onMowArea(e.key),
                    }}
                >
                    {t("mapToolbar.mowArea")}
                </AsyncDropDownButton>
            </Tooltip>

            <Tooltip
                title={tooltipTitle(manualMode ? "mapToolbar.stopManualTooltip" : "mapToolbar.manualMowTooltip")}
            >
                <AsyncButton
                    danger={manualMode}
                    icon={manualMode ? <HomeOutlined /> : <ControlOutlined />}
                    onAsyncClick={manualMode ? onStopManualMode : onManualMode}
                >
                    {manualMode ? t("mapToolbar.stopManual") : t("mapToolbar.manualMow")}
                </AsyncButton>
            </Tooltip>

            <Dropdown
                menu={{items: moreMenuItems, onClick: handleMoreClick}}
                trigger={["click"]}
            >
                <Tooltip title={tooltipTitle("mapToolbar.moreTooltip")}>
                    <Button icon={<EllipsisOutlined />}>{t("mapToolbar.more")}</Button>
                </Tooltip>
            </Dropdown>
        </Space>
    );
};
