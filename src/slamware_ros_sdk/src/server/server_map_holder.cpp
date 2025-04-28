#include "server_map_holder.h"
#include <cassert>

#include <cmath>
#include <cstring>
#include <stdexcept>

namespace slamware_ros_sdk {

    const float ServerMapHolder::C_DEFAULT_RESOLUTION = 0.05f;
    const std::uint32_t ServerMapHolder::C_MAP_DATA_SIZE_ALIGNMENT = 16u;
    const int ServerMapHolder::C_DEFAULT_MORE_CELL_CNT_TO_EXTEND = 32;

    ServerMapHolder::ServerMapHolder()
        : moreCellCntToExtend_(C_DEFAULT_MORE_CELL_CNT_TO_EXTEND)
        , resolution_(C_DEFAULT_RESOLUTION)
    {
        //
    }

    ServerMapHolder::~ServerMapHolder()
    {
        //
    }

    void ServerMapHolder::clear()
    {
        availCellIdxRect_ = utils::RectangleI();
        availMapArea_ = utils::RectangleF();
        validCellIdxRect_ = utils::RectangleI();
        validMapArea_ = utils::RectangleF();
        map_data_t().swap(mapDat_);
    }

    void ServerMapHolder::reinit(float resolution)
    {
        sfCheckResolutionValidity_(resolution);

        clear();
        resolution_ = resolution;
    }

    void ServerMapHolder::setMoreCellCountToExtend(int moreCellCntToExtend)
    {
        moreCellCntToExtend_ = (0 <= moreCellCntToExtend ? moreCellCntToExtend : C_DEFAULT_MORE_CELL_CNT_TO_EXTEND);
    }

    void ServerMapHolder::reserveByCellIdxRect(const utils::RectangleI& cellIdxRect)
    {
        if (sfIsCellIdxRectEmpty(cellIdxRect))
            return;
        if (sfDoesCellIdxRectContain(availCellIdxRect_, cellIdxRect))
            return;

        auto newAvailCellIdxRect = sfMergeCellIdxRect(availCellIdxRect_, cellIdxRect);
        newAvailCellIdxRect = sfCalcAdjustedCellIdxRect_(newAvailCellIdxRect);

        const size_t newDatSize = newAvailCellIdxRect.width() * newAvailCellIdxRect.height();
        map_data_t newMapDat(newDatSize);
        std::memset(newMapDat.data(), 0, (newDatSize * sizeof(cell_value_t)));

        if (!sfIsCellIdxRectEmpty(validCellIdxRect_))
        {
            const int destOffX = validCellIdxRect_.x() - newAvailCellIdxRect.x();
            assert(0 <= destOffX && destOffX < newAvailCellIdxRect.width());
            assert(destOffX + validCellIdxRect_.width() <= newAvailCellIdxRect.width());

            const int destOffY = validCellIdxRect_.y() - newAvailCellIdxRect.y();
            assert(0 <= destOffY && destOffY < newAvailCellIdxRect.height());
            assert(destOffY + validCellIdxRect_.height() <= newAvailCellIdxRect.height());

            const size_t srcBytesPerLine = validCellIdxRect_.width() * sizeof(cell_value_t);
            size_t srcCellOffset = (validCellIdxRect_.y() - availCellIdxRect_.y()) * availCellIdxRect_.width() + (validCellIdxRect_.x() - availCellIdxRect_.x());
            size_t destCellOffset = destOffY * newAvailCellIdxRect.width() + destOffX;
            for (int j = 0; j < validCellIdxRect_.height(); ++j)
            {
                std::memcpy(&newMapDat[destCellOffset], &mapDat_[srcCellOffset], srcBytesPerLine);

                srcCellOffset += availCellIdxRect_.width();
                destCellOffset += newAvailCellIdxRect.width();
            }
        }

        availCellIdxRect_ = newAvailCellIdxRect;
        availMapArea_ = calcAreaByCellIdxRect(availCellIdxRect_);
        mapDat_.swap(newMapDat);

    #if 0
        RCLCPP_INFO(node->get_logger(), "ServerMapHolder::reserveByCellIdxRect(), avail rect: ((%d, %d), (%d, %d)), avail area: ((%f, %f), (%f, %f))."
            , availCellIdxRect_.x(), availCellIdxRect_.y(), availCellIdxRect_.width(), availCellIdxRect_.height()
            , availMapArea_.x(), availMapArea_.y(), availMapArea_.width(), availMapArea_.height()
            );
    #endif
    }

    void ServerMapHolder::reserveByArea(const utils::RectangleF& reqArea)
    {
        if (reqArea.empty())
            return;

        const auto minBoundingCellIdxRect = calcMinBoundingCellIdxRect(reqArea);
        reserveByCellIdxRect(minBoundingCellIdxRect);
    }

    void ServerMapHolder::setMapData(float x, float y, float resolution, int dimensionX, int dimensionY, const cell_value_t* srcDat)
    {
        if (dimensionX < 1 || dimensionY < 1)
            return;
        assert(nullptr != srcDat);

        sfCheckResolutionEquality_(resolution, resolution_);

        const utils::RectangleI cellIdxRectToUp(static_cast<int>(std::round(x / resolution_))
            , static_cast<int>(std::round(y / resolution_))
            , dimensionX
            , dimensionY
            );
        checkToExtendMap_(cellIdxRectToUp);

        const int destOffX = cellIdxRectToUp.x() - availCellIdxRect_.x();
        assert(0 <= destOffX && destOffX < availCellIdxRect_.width());
        assert(destOffX + dimensionX <= availCellIdxRect_.width());

        const int destOffY = cellIdxRectToUp.y() - availCellIdxRect_.y();
        assert(0 <= destOffY && destOffY < availCellIdxRect_.height());
        assert(destOffY + dimensionY <= availCellIdxRect_.height());

        const size_t srcBytesPerLine = dimensionX * sizeof(cell_value_t);
        size_t srcCellOffset = 0;
        size_t destCellOffset = destOffY * availCellIdxRect_.width() + destOffX;
        for (int j = 0; j < dimensionY; ++j)
        {
            std::memcpy(&mapDat_[destCellOffset], &srcDat[srcCellOffset], srcBytesPerLine);

            srcCellOffset += dimensionX;
            destCellOffset += availCellIdxRect_.width();
        }

        validCellIdxRect_ = sfMergeCellIdxRect(validCellIdxRect_, cellIdxRectToUp);
        validMapArea_ = calcAreaByCellIdxRect(validCellIdxRect_);
    }

    utils::RectangleI ServerMapHolder::fillRosMapMsg(const utils::RectangleI& reqIdxRect, nav_msgs::srv::GetMap::Response& msgMap) const
    {
        const auto resIdxRect = reqIdxRect;
        const auto resArea = calcAreaByCellIdxRect(resIdxRect);

        const auto intersecRect = sfIntersectionOfCellIdxRect(validCellIdxRect_, resIdxRect);

        msgMap.map.info.resolution = resolution_;
        msgMap.map.info.origin.position.x = resArea.x();
        msgMap.map.info.origin.position.y = resArea.y();
        msgMap.map.info.origin.position.z = 0.0;
        msgMap.map.info.origin.orientation.x = 0.0;
        msgMap.map.info.origin.orientation.y = 0.0;
        msgMap.map.info.origin.orientation.z = 0.0;
        msgMap.map.info.origin.orientation.w = 1.0;

        msgMap.map.info.width = 0;
        msgMap.map.info.height = 0;
        msgMap.map.data.clear();
        if (!sfIsCellIdxRectEmpty(resIdxRect))
        {
            msgMap.map.info.width = resIdxRect.width();
            msgMap.map.info.height = resIdxRect.height();
            const size_t destDatSize = msgMap.map.info.width * msgMap.map.info.height;
            msgMap.map.data.resize(destDatSize, -1);
            //
            if (!sfIsCellIdxRectEmpty(intersecRect))
            {
                const int srcOffsetX = intersecRect.x() - availCellIdxRect_.x();
                const int srcOffsetY = intersecRect.y() - availCellIdxRect_.y();
                size_t srcRowOffset = srcOffsetY * availCellIdxRect_.width() + srcOffsetX;
                //
                const int destOffsetX = intersecRect.x() - resIdxRect.x();
                const int destOffsetY = intersecRect.y() - resIdxRect.y();
                size_t destRowOffset = destOffsetY * resIdxRect.width() + destOffsetX;
                //
                for (int j = 0; j < intersecRect.height(); ++j)
                {
                    size_t tSrcIdx = srcRowOffset;
                    size_t tDestIdx = destRowOffset;
                    for (int i = 0; i < intersecRect.width(); ++i)
                    {
                        const auto tVal = mapDat_[tSrcIdx];
                        if (0 == tVal)
                            msgMap.map.data[tDestIdx] = -1;  // unknown
                        else if (tVal <= 127)
                            msgMap.map.data[tDestIdx] = 0;  // free
                        else if (127 < tVal)
                            msgMap.map.data[tDestIdx] = 100;  // occupied
                        //
                        ++tSrcIdx;
                        ++tDestIdx;
                    }
                    //
                    srcRowOffset += availCellIdxRect_.width();
                    destRowOffset += resIdxRect.width();
                }
            }
        }

        return resIdxRect;
    }

    utils::RectangleI ServerMapHolder::fillRosMapMsg(const utils::RectangleF& reqArea, nav_msgs::srv::GetMap::Response& msgMap) const
    {
        utils::RectangleI reqIdxRect;
        if (!reqArea.empty())
            reqIdxRect = calcMinBoundingCellIdxRect(reqArea);
        return fillRosMapMsg(reqIdxRect, msgMap);
    }

    utils::RectangleI ServerMapHolder::fillRosMapMsg(nav_msgs::srv::GetMap::Response& msgMap) const
    {
        return fillRosMapMsg(validCellIdxRect_, msgMap);
    }

    bool ServerMapHolder::sfDoesCellIdxRectContain(const utils::RectangleI& cellIdxRect, const utils::RectangleI& objRect)
    {
        if (sfIsCellIdxRectEmpty(cellIdxRect))
            return false;
        const int xEnd = cellIdxRect.x() + cellIdxRect.width();
        const int yEnd = cellIdxRect.y() + cellIdxRect.height();

        const int objXEnd = (0 < objRect.width() ? (objRect.x() + objRect.width()) : objRect.x());
        const int objYEnd = (0 < objRect.height() ? (objRect.y() + objRect.height()) : objRect.y());
        return (cellIdxRect.x() <= objRect.x() && objRect.x() < xEnd
            && objXEnd <= xEnd
            && cellIdxRect.y() <= objRect.y() && objRect.y() < yEnd
            && objYEnd <= yEnd
            );
    }

    utils::RectangleI ServerMapHolder::sfMergeCellIdxRect(const utils::RectangleI& idxRectA, const utils::RectangleI& idxRectB)
    {
        if (sfIsCellIdxRectEmpty(idxRectB))
            return idxRectA;
        if (sfIsCellIdxRectEmpty(idxRectA))
            return idxRectB;

        const int xMin = std::min<int>(idxRectA.x(), idxRectB.x());
        const int yMin = std::min<int>(idxRectA.y(), idxRectB.y());

        const int aXEnd = idxRectA.x() + idxRectA.width();
        const int aYEnd = idxRectA.y() + idxRectA.height();
        const int bXEnd = idxRectB.x() + idxRectB.width();
        const int bYEnd = idxRectB.y() + idxRectB.height();
        const int xEnd = std::max<int>(aXEnd, bXEnd);
        const int yEnd = std::max<int>(aYEnd, bYEnd);

        assert(xMin < xEnd);
        assert(yMin < yEnd);
        return utils::RectangleI(xMin
            , yMin
            , (xEnd - xMin)
            , (yEnd - yMin)
            );
    }

    utils::RectangleI ServerMapHolder::sfIntersectionOfCellIdxRect(const utils::RectangleI& idxRectA, const utils::RectangleI& idxRectB)
    {
        if (sfIsCellIdxRectEmpty(idxRectA)
            || sfIsCellIdxRectEmpty(idxRectB)
            )
        {
            return utils::RectangleI();
        }

        const int xMin = std::max<int>(idxRectA.x(), idxRectB.x());
        const int yMin = std::max<int>(idxRectA.y(), idxRectB.y());

        const int aXEnd = idxRectA.x() + idxRectA.width();
        const int aYEnd = idxRectA.y() + idxRectA.height();
        const int bXEnd = idxRectB.x() + idxRectB.width();
        const int bYEnd = idxRectB.y() + idxRectB.height();
        const int xEnd = std::min<int>(aXEnd, bXEnd);
        const int yEnd = std::min<int>(aYEnd, bYEnd);

        return utils::RectangleI(xMin
            , yMin
            , (xMin < xEnd ? (xEnd - xMin) : 0)
            , (yMin < yEnd ? (yEnd - yMin) : 0)
            );
    }

    utils::RectangleF ServerMapHolder::sfCalcAreaByCellIdxRect(float resolution, const utils::RectangleI& cellIdxRect)
    {
        assert(FLT_EPSILON < resolution);
        return utils::RectangleF(resolution * cellIdxRect.x()
            , resolution * cellIdxRect.y()
            , resolution * cellIdxRect.width()
            , resolution * cellIdxRect.height()
            );
    }

    utils::RectangleI ServerMapHolder::sfCalcMinBoundingCellIdxRect(float resolution, const utils::RectangleF& reqArea)
    {
        assert(FLT_EPSILON < resolution);
        
        int idxXMin = static_cast<int>(std::floor(reqArea.x() / resolution));
        if (reqArea.x() < resolution * idxXMin)
            --idxXMin;

        int idxYMin = static_cast<int>(std::floor(reqArea.y() / resolution));
        if (reqArea.y() < resolution * idxYMin)
            --idxYMin;

        const float srcXMax = (0.0f < reqArea.width() ? (reqArea.x() + reqArea.width()) : reqArea.x());
        int idxXMax = static_cast<int>(std::floor(srcXMax / resolution));
        if (resolution * (idxXMax + 1) < srcXMax)
            ++idxXMax;

        const float srcYMax = (0.0f < reqArea.height() ? (reqArea.y() + reqArea.height()) : reqArea.y());
        int idxYMax = static_cast<int>(std::floor(srcYMax / resolution));
        if (resolution * (idxYMax + 1) < srcYMax)
            ++idxYMax;

        assert(idxXMin <= idxXMax);
        assert(idxYMin <= idxYMax);
        return utils::RectangleI(idxXMin
            , idxYMin
            , (idxXMax - idxXMin + 1)
            , (idxYMax - idxYMin + 1)
            );
    }

    utils::RectangleF ServerMapHolder::sfCalcMinBoundingArea(float resolution, const utils::RectangleF& reqArea)
    {
        assert(FLT_EPSILON < resolution);
        const auto minBoundingCellIdxRect = sfCalcMinBoundingCellIdxRect(resolution, reqArea);
        return sfCalcAreaByCellIdxRect(resolution, minBoundingCellIdxRect);
    }

    utils::RectangleI ServerMapHolder::sfCalcRoundedCellIdxRect(float resolution, const utils::RectangleF& reqArea)
    {
        assert(FLT_EPSILON < resolution);
        return utils::RectangleI(static_cast<int>(std::round(reqArea.x() / resolution))
            , static_cast<int>(std::round(reqArea.y() / resolution))
            , static_cast<int>(std::round(reqArea.width() / resolution))
            , static_cast<int>(std::round(reqArea.height() / resolution))
            );
    }

    utils::RectangleF ServerMapHolder::sfCalcRoundedArea(float resolution, const utils::RectangleF& reqArea)
    {
        assert(FLT_EPSILON < resolution);
        const auto roundedCellIdxRect = sfCalcRoundedCellIdxRect(resolution, reqArea);
        return sfCalcAreaByCellIdxRect(resolution, roundedCellIdxRect);
    }

    void ServerMapHolder::checkToExtendMap_(const utils::RectangleI& cellIdxRectToUp)
    {
        assert(0 <= moreCellCntToExtend_);
        assert(0 <= availCellIdxRect_.width());
        assert(0 <= availCellIdxRect_.height());

        if (sfIsCellIdxRectEmpty(cellIdxRectToUp))
            return;
        if (sfDoesCellIdxRectContain(availCellIdxRect_, cellIdxRectToUp))
            return;

        int tXMin = availCellIdxRect_.x();
        if (cellIdxRectToUp.x() < tXMin)
            tXMin = cellIdxRectToUp.x() - moreCellCntToExtend_;

        int tYMin = availCellIdxRect_.y();
        if (cellIdxRectToUp.y() < tYMin)
            tYMin = cellIdxRectToUp.y() - moreCellCntToExtend_;

        const int upXEnd = cellIdxRectToUp.x() + cellIdxRectToUp.width();
        int tXEnd = availCellIdxRect_.x() + availCellIdxRect_.width();
        if (tXEnd < upXEnd)
            tXEnd = upXEnd + moreCellCntToExtend_;

        const int upYEnd = cellIdxRectToUp.y() + cellIdxRectToUp.height();
        int tYEnd = availCellIdxRect_.y() + availCellIdxRect_.height();
        if (tYEnd < upYEnd)
            tYEnd = upYEnd + moreCellCntToExtend_;

        assert(tXMin < tXEnd);
        assert(tYMin < tYEnd);
        const auto tCellIdxRect = utils::RectangleI(tXMin
            , tYMin
            , (tXEnd - tXMin)
            , (tYEnd - tYMin)
            );
        reserveByCellIdxRect(tCellIdxRect);
    }

    void ServerMapHolder::sfCheckResolutionValidity_(float resolution)
    {
        if (resolution <= FLT_EPSILON)
            throw std::runtime_error("invalid resolution.");
    }

    void ServerMapHolder::sfCheckResolutionEquality_(float resA, float resB)
    {
        if (std::abs(resA - resB) > FLT_EPSILON)
          throw std::runtime_error("inconsistent resolution.");
        if (resA <= FLT_EPSILON)
            throw std::runtime_error("invalid resolution.");
        if (resB <= FLT_EPSILON)
            throw std::runtime_error("invalid resolution.");
    }

    int ServerMapHolder::sfGetAlignedFloorCellIdx_(int inCellIdx)
    {
        if (inCellIdx < 0)
        {
            std::uint32_t tmpIdx = static_cast<std::uint32_t>(-inCellIdx);
            --tmpIdx;
            tmpIdx /= C_MAP_DATA_SIZE_ALIGNMENT;
            ++tmpIdx;
            tmpIdx *= C_MAP_DATA_SIZE_ALIGNMENT;
            return (-static_cast<int>(tmpIdx));
        }
        else
        {
            std::uint32_t tmpIdx = static_cast<std::uint32_t>(inCellIdx);
            tmpIdx /= C_MAP_DATA_SIZE_ALIGNMENT;
            tmpIdx *= C_MAP_DATA_SIZE_ALIGNMENT;
            return static_cast<int>(tmpIdx);
        }
    }

    int ServerMapHolder::sfGetAlignedCeilSize_(int inSize)
    {
        if (inSize < 0)
            throw std::runtime_error("invalid input size.");

        std::uint32_t tmpSize = static_cast<std::uint32_t>(inSize);
        tmpSize += (C_MAP_DATA_SIZE_ALIGNMENT - 1);
        tmpSize /= C_MAP_DATA_SIZE_ALIGNMENT;
        tmpSize *= C_MAP_DATA_SIZE_ALIGNMENT;
        return static_cast<int>(tmpSize);
    }
    
    utils::RectangleI ServerMapHolder::sfCalcAdjustedCellIdxRect_(const utils::RectangleI& idxRect)
    {
        if (sfIsCellIdxRectEmpty(idxRect))
            return utils::RectangleI();

        const int xMin = sfGetAlignedFloorCellIdx_(idxRect.x());
        assert(xMin <= idxRect.x());
        const int yMin = sfGetAlignedFloorCellIdx_(idxRect.y());
        assert(yMin <= idxRect.y());

        const int reqXEnd = idxRect.x() + idxRect.width();
        const int reqYEnd = idxRect.y() + idxRect.height();
        const int tWidth = sfGetAlignedCeilSize_(reqXEnd - xMin);
        assert(reqXEnd <= xMin + tWidth);
        const int tHeight = sfGetAlignedCeilSize_(reqYEnd - yMin);
        assert(reqYEnd <= yMin + tHeight);

        return utils::RectangleI(xMin, yMin, tWidth, tHeight);
    }

}
