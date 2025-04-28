
#pragma once

#include "geometry_rectangle.h"
#include "nav_msgs/srv/get_map.hpp"
#include "rclcpp/rclcpp.hpp"

#include <cstdint>

namespace slamware_ros_sdk {

    class ServerMapHolder
    {
    public:
        static const float C_DEFAULT_RESOLUTION;
        static const std::uint32_t C_MAP_DATA_SIZE_ALIGNMENT;
        static const int C_DEFAULT_MORE_CELL_CNT_TO_EXTEND;

        typedef std::uint8_t                    cell_value_t;
        typedef std::vector<cell_value_t>       map_data_t;

    public:
        ServerMapHolder();
        ~ServerMapHolder();

        float resolution() const { return resolution_; }
        bool isMapDataEmpty() const { return sfIsCellIdxRectEmpty(validCellIdxRect_); }

        // when extending rect of map data, it will extend more count of cells to prevent memory allocation frequently.
        int getMoreCellCountToExtend() const { return moreCellCntToExtend_; }

        void clear();
        void reinit(float resolution);

        void setMoreCellCountToExtend(int moreCellCntToExtend);

        void reserveByCellIdxRect(const utils::RectangleI& cellIdxRect);
        void reserveByArea(const utils::RectangleF& reqArea);

        void setMapData(float x, float y, float resolution, int dimensionX, int dimensionY, const cell_value_t* srcDat);

    public:
        const utils::RectangleI& getValidCellIdxRect() const { return validCellIdxRect_; }
        const utils::RectangleF& getValidMapArea() const { return validMapArea_; }

        // returns the cell index rect of actually filled cells
        utils::RectangleI fillRosMapMsg(const utils::RectangleI& reqIdxRect, nav_msgs::srv::GetMap::Response& msgMap) const;
        utils::RectangleI fillRosMapMsg(const utils::RectangleF& reqArea, nav_msgs::srv::GetMap::Response& msgMap) const;
        utils::RectangleI fillRosMapMsg(nav_msgs::srv::GetMap::Response& msgMap) const;

    public:
        utils::RectangleF calcAreaByCellIdxRect(const utils::RectangleI& cellIdxRect) const { return sfCalcAreaByCellIdxRect(resolution_, cellIdxRect); }

        utils::RectangleI calcMinBoundingCellIdxRect(const utils::RectangleF& reqArea) const { return sfCalcMinBoundingCellIdxRect(resolution_, reqArea); }
        utils::RectangleF calcMinBoundingArea(const utils::RectangleF& reqArea) const { return sfCalcMinBoundingArea(resolution_, reqArea); }

        utils::RectangleI calcRoundedCellIdxRect(const utils::RectangleF& reqArea) const { return sfCalcRoundedCellIdxRect(resolution_, reqArea); }
        utils::RectangleF calcRoundedArea(const utils::RectangleF& reqArea) const { return sfCalcRoundedArea(resolution_, reqArea); }

    public:
        static bool sfIsCellIdxRectEmpty(const utils::RectangleI& cellIdxRect)
        {
            return (cellIdxRect.width() <= 0 || cellIdxRect.height() <= 0);
        }
        static bool sfDoesCellIdxRectContain(const utils::RectangleI& cellIdxRect, int cellIdxX, int cellIdxY)
        {
            return (0 < cellIdxRect.width() && 0 < cellIdxRect.height()
                && cellIdxRect.x() <= cellIdxX && cellIdxX < (cellIdxRect.x() + cellIdxRect.width())
                && cellIdxRect.y() <= cellIdxY && cellIdxY < (cellIdxRect.y() + cellIdxRect.height())
                );
        }
        static bool sfDoesCellIdxRectContain(const utils::RectangleI& cellIdxRect, const utils::RectangleI& objRect);
        static utils::RectangleI sfMergeCellIdxRect(const utils::RectangleI& idxRectA, const utils::RectangleI& idxRectB);
        static utils::RectangleI sfIntersectionOfCellIdxRect(const utils::RectangleI& idxRectA, const utils::RectangleI& idxRectB);

        static utils::RectangleF sfCalcAreaByCellIdxRect(float resolution, const utils::RectangleI& cellIdxRect);

        static utils::RectangleI sfCalcMinBoundingCellIdxRect(float resolution, const utils::RectangleF& reqArea);
        static utils::RectangleF sfCalcMinBoundingArea(float resolution, const utils::RectangleF& reqArea);

        static utils::RectangleI sfCalcRoundedCellIdxRect(float resolution, const utils::RectangleF& reqArea);
        static utils::RectangleF sfCalcRoundedArea(float resolution, const utils::RectangleF& reqArea);

    private:
        void checkToExtendMap_(const utils::RectangleI& cellIdxRectToUp);

    private:
        static void sfCheckResolutionValidity_(float resolution);
        static void sfCheckResolutionEquality_(float resA, float resB);

        // returns aligned floor cell index, may be negative.
        static int sfGetAlignedFloorCellIdx_(int inCellIdx);
        // returns aligned ceil size (>= 0).
        static int sfGetAlignedCeilSize_(int inSize);
        
        // returns cell index rect that position is aligned floor and size is aligned ceil.
        static utils::RectangleI sfCalcAdjustedCellIdxRect_(const utils::RectangleI& idxRect);

    private:
        int moreCellCntToExtend_;

        float resolution_;
        utils::RectangleI availCellIdxRect_;
        utils::RectangleF availMapArea_;
        utils::RectangleI validCellIdxRect_;
        utils::RectangleF validMapArea_;
        map_data_t mapDat_;
    };

}
