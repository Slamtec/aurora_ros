/*
* geometry_rectangle.h
* Rectangle objects
*/

#pragma once
#include <algorithm>
#include <cfloat>

namespace slamware_ros_sdk { namespace utils {

	template<typename T>
    class Vector2 {
    public:
        typedef T Scalar;
        Vector2() : x_(0), y_(0) {}
        Vector2(T x, T y) : x_(x), y_(y) {}
            
        T x() const { return x_; }
        T& x() { return x_; }
        T y() const { return y_; }
        T& y() { return y_; }

        Vector2 operator+(const Vector2& other) const {
            return Vector2(x_ + other.x_, y_ + other.y_);
        }

        bool operator==(const Vector2& other) const {
            return x_ == other.x_ && y_ == other.y_;
        }

        bool operator!=(const Vector2& other) const {
            return !(*this == other);
        }

    private:
        T x_, y_;
    };

    typedef Vector2<float> Vector2f;
    typedef Vector2<int> Vector2i;

    namespace detail {
        template<class VectorT>
        class _Rectangle {
        public:
            typedef typename VectorT::Scalar scalar_t;

            _Rectangle()
                : position_(0,0), size_(0,0)
            {}

            _Rectangle(VectorT position, VectorT size)
                : position_(position), size_(size)
            {}

            _Rectangle(scalar_t x, scalar_t y, scalar_t width, scalar_t height)
                : position_(x, y), size_(width, height)
            {}

            _Rectangle(const _Rectangle& that)
                : position_(that.position_), size_(that.size_)
            {}

            ~_Rectangle()
            {}

        public:
            _Rectangle& operator=(const _Rectangle& that)
            {
                position_ = that.position_;
                size_ = that.size_;

                return *this;
            }

        public:
            const VectorT& position() const
            {
                return position_;
            }

            VectorT& position()
            {
                return position_;
            }

            const VectorT& size() const
            {
                return size_;
            }

            VectorT& size()
            {
                return size_;
            }

            scalar_t x() const
            {
                return position_.x();
            }

            scalar_t& x()
            {
                return position_.x();
            }


            scalar_t y() const
            {
                return position_.y();
            }

            scalar_t& y()
            {
                return position_.y();
            }


            scalar_t width() const
            {
                return size_.x();
            }

            scalar_t& width()
            {
                return size_.x();
            }

            scalar_t height() const
            {
                return size_.y();
            }

            scalar_t& height()
            {
                return size_.y();
            }

            scalar_t left() const
            {
                return position_.x();
            }

            scalar_t right() const
            {
                return position_.x() + size_.x();
            }

            scalar_t top() const
            {
                return position_.y();
            }

            scalar_t bottom() const
            {
                return position_.y() + size_.y();
            }

            bool contains(const VectorT & point) const
            {
                if (point.x() < x() || point.x() >= right()) return false;
                if (point.y() < y() || point.y() >= bottom()) return false;
                return true;
            }

            bool contains(float px, float py) const
            {
                if (px < x() || px >= right()) return false;
                if (py < y() || py >= bottom()) return false;
                return true;
            }

            bool empty() const
            {
                return (size_.x() <= (scalar_t)FLT_EPSILON && size_.y() <= (scalar_t)FLT_EPSILON);
            }

            bool contains(const _Rectangle & dest) const
            {
                return contains(dest.position()) && contains(dest.position() + dest.size());
            }

            void unionOf(const _Rectangle & dest)
            {
                if (empty()) {
                    *this = dest;
                    return ;
                }

                float new_left = std::min<float>(dest.x(), x());
                float new_top = std::min<float>(dest.y(), y());

                float new_right = std::max<float>(dest.right(), right());
                float new_bottom = std::max<float>(dest.bottom(), bottom());

                x() = new_left;
                y() = new_top;

                width() =  new_right - new_left;
                height() = new_bottom - new_top;
            }

            bool isIntersectionWith(const _Rectangle & dest) const
            {
                if (empty())
                    return false;
                if (left() >= dest.right() || right() <= dest.left() || top() >= dest.bottom() || bottom() <= dest.top())
                    return false;
                return true;
            }

            void intersectionOf(const _Rectangle & dest)
            {
                if (empty()) return;

                float new_left = std::max<float>(dest.x(), x());
                float new_top = std::max<float>(dest.y(), y());

                float new_right = std::min<float>(dest.right(), right());
                float new_bottom = std::min<float>(dest.bottom(), bottom());               

                if ( new_right <= new_left || new_bottom <= new_top)
                {
                    x() = 0;
                    y() = 0;
                    width() = 0;
                    height() = 0;
                } else {
                    x() = new_left;
                    y() = new_top;

                    width() =  new_right - new_left;
                    height() = new_bottom - new_top;
                }
            }
                
            scalar_t area() const
            {
                return size_.x() * size_.y();
            }

        private:
            VectorT position_, size_;
        };
            
        template<typename VectorT>
        bool operator==(const _Rectangle<VectorT>& a, const _Rectangle<VectorT>& b)
        {
            return a.position() == b.position() && a.size() == b.size();
        }
            
        template<typename VectorT>
        bool operator!=(const _Rectangle<VectorT>& a, const _Rectangle<VectorT>& b)
        {
            return a.position() != b.position() || a.size() != b.size();
        }
    }

    typedef detail::_Rectangle<Vector2f> RectangleF;
    typedef detail::_Rectangle<Vector2i> RectangleI;

} }
