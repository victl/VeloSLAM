#ifndef __TimeLine_H__
#define __TimeLine_H__

#include <boost/date_time.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/circular_buffer.hpp>
#include <vector>
#include <utility>
#include <cmath>
#include <glog/logging.h>
#include <inttypes.h>

using namespace boost::posix_time;
using namespace boost::gregorian;
using namespace std;

template <typename T_>
class TimeLine {
public:
    TimeLine()
        : interval(0)
        , buffer(5)
        , timelineCursor(0)
        , interCursor(0)
        , total_num(0)
        , intervalFinalized(false)
    {

    }

    ~TimeLine(){

    }

    size_t size() {
        return total_num;
    }

    boost::shared_ptr<T_> back() {
        if (timeline.empty())
            return boost::shared_ptr<T_>();
        return timeline.back().back();
    }

    void addData(boost::shared_ptr<T_> data);

    boost::shared_ptr<T_> getExactDataAt(const ptime& t);

    /* for all getNearest***(), you need to check for the validity of return
     * value to make sure queryed time is valid */
    ptime getNearestTime(const ptime& t, bool shouldSetCursor = false);

    ptime getNearestTime(const boost::shared_ptr<T_ > &data);

    /* for all getBoundary***(), you need to check the validity of return value
     * to make sure queryed time lies with 'startTime' and 'maxTime' */
    pair<ptime, ptime> getBoundaryTime(const ptime& t);

    pair<ptime, ptime> getBoundaryTime(const boost::shared_ptr<T_ >& data);

    boost::shared_ptr<T_> getNearestData(const ptime& t, bool shouldSetCursor = false);

    boost::shared_ptr<T_> getNearestData(const boost::shared_ptr<T_>& data);

    pair<boost::shared_ptr<T_ >, boost::shared_ptr<T_ > > getBoundaryData(const ptime& t);

    pair<boost::shared_ptr<T_ >, boost::shared_ptr<T_ > > getBoundaryData(
            const boost::shared_ptr<T_ > &data);

    vector<boost::shared_ptr<T_>> getRangeBetween(const ptime& a, const ptime& b);
    vector<boost::shared_ptr<T_>> getAll();

    // set the cursor to the nearest time to 't', then you could start a sequential
    // query for data (via this->nextData()
    bool setCursorTo(boost::posix_time::ptime& t);

    void resetCursor() {
        timelineCursor = interCursor = 0;
    }

    /* best practice is to use "
     *
     * obj.resetCursor();
     * do {
     * ...nexData()...
     * } while(obj.hasMore());
     *
     * " loop structure
     */
    bool cursorAtFront() const {
        return timelineCursor == 0 && interCursor == 0;
    }

    // in-mature, don't use it!!!
    boost::shared_ptr<T_> nextData();

    void clear() {
        timeline.clear();
        buffer.clear();
        startTime = maxTime = ptime();
        timelineCursor = interCursor = 0;
        interval = 0;
        total_num = 0;
    }

    void unload() {
        clear();
        vector<std::vector<boost::shared_ptr<T_>>> newtimeline;
        boost::circular_buffer<boost::shared_ptr<T_>> newbuffer;
        timeline.swap(newtimeline);
        buffer.swap(newbuffer);
    }


private:
    std::vector<std::vector<boost::shared_ptr<T_>>> timeline;
    /* Because most querys happen at the end of the timeline, so
     * using a circular_buffer could greatly improve the search time */
    boost::circular_buffer<boost::shared_ptr<T_>> buffer;
    ptime startTime, maxTime;
    double interval;
    bool intervalFinalized;
    int timelineCursor, interCursor;
    size_t total_num;

    void calcNewAvgIntervalAndRearrange();

    boost::shared_ptr<T_> searchVec(const std::vector<boost::shared_ptr<T_ > > &vec
            , const ptime& t
            , long& shortest);
};

template <typename T_>
void TimeLine<T_>::addData(boost::shared_ptr<T_ > data)
{
    ptime& t = data->timestamp;
    if (timeline.empty()) {
        startTime = t;
        maxTime = t;
        timeline.push_back(std::vector<boost::shared_ptr<T_>>());
        timeline.back().push_back(data);
        buffer.push_back(data);
        ++ total_num;
    } else if (timeline.size() == 1) {
        if (t == startTime) {
            DLOG(WARNING) << "[WARNING]: You inserted a data that is already inside the container.\n"
                         "Old data will be OVERWRITTEN!!";
            timeline.back().back() = data;
            buffer[0] = data;
            return;
        } else if (t < startTime) { // This sould be the very rare case
            interval = (startTime - t).total_microseconds() * 0.95;
            timeline.insert(timeline.begin(), std::vector<boost::shared_ptr<T_>>());
            timeline[0].push_back(data);
            buffer.push_front(data);
            maxTime = startTime;
            startTime = t;
            ++ total_num;
            return;
        } else {
            interval = (t - startTime).total_microseconds() * 0.95;
            timeline.push_back(std::vector<boost::shared_ptr<T_>>());
            timeline.back().push_back(data);
            buffer.push_back(data);
            maxTime = t;
            ++ total_num;
        }
    } else { // timeline.size() >= 2
        //deal with buffer first
        if ((!intervalFinalized) && total_num == 10) calcNewAvgIntervalAndRearrange();
        if (t > buffer.back()->timestamp) {
            buffer.push_back(data);
        } else {
            int cursor = 0;
            while(buffer[cursor]->timestamp < t) ++cursor;
            if (buffer[cursor]->timestamp == t) {
                DLOG(WARNING) << "[WARNING]: You inserted a data that is already inside the container.\n"
                             "Old data will be OVERWRITTEN!!";
                buffer[cursor] = data;
            } else {
                buffer.insert(buffer.begin() + cursor, data);
            }
        } // done with buffer
        int index = (t - startTime).total_microseconds() / interval;
        if (t >= startTime) { // then index should be a Natural Number (not negetive)
            while (timeline.size() <= index) { // fill the timeline 'gap'
                timeline.push_back(std::vector<boost::shared_ptr<T_>>());
            }
            // Insert into timeline
            int insertPos = 0;
            while (insertPos < timeline[index].size() && timeline[index][insertPos]->timestamp < t)
                ++insertPos;
            if (insertPos == timeline[index].size()) {
                        timeline[index].push_back(data);
                        maxTime = t;
                        ++ total_num;
            }else if (timeline[index][insertPos]->timestamp == t) {
                DLOG(WARNING) << "[WARNING]: You inserted a data that is already inside the container.\n"
                             "Old data will be OVERWRITTEN!!";
                timeline[index][insertPos] = data;
            }  else {
                //NOT THREAD SAFE
                DLOG(WARNING) << "[WARNING]: Insertion in the middle of the timeline could result.\n"
                             "in unsafe multithread behaviour.";
                timeline[index].insert(timeline[index].begin() + insertPos, data);
                ++ total_num;
            }
        } else {
            // index will be a negative number
            DLOG(WARNING) << "[WARNING]: You inserted a data that is older than startTime.\n"
                         "This could be very slow!!";
            index = floor((t - startTime).total_microseconds() / interval);
            while((index++) != 0) // fill the gap
                timeline.insert(timeline.begin(), std::vector<boost::shared_ptr<T_>>());
            timeline.front().push_back(data);
            startTime = t;
            ++ total_num;
        }
    }
}

template <typename T_>
boost::shared_ptr<T_> TimeLine<T_>::getExactDataAt(const ptime &t)
{
    boost::shared_ptr<T_> invalid_ptr;
    if (t > maxTime || t < startTime) {
        return invalid_ptr;
    }
    int index = (t - startTime).total_microseconds() / interval;
    if (timeline[index].empty()){
        return invalid_ptr;
    }
    for (auto& obj : timeline[index]) {
        if (t == obj->timestamp) {
            return obj;
        }
    }
    return invalid_ptr;
}

template <typename T_>
ptime TimeLine<T_>::getNearestTime(const ptime &t, bool shouldSetCursor)
{
    auto d = getNearestData(t, shouldSetCursor);
    if (d)
        return d->timestamp;
    else
        return ptime();
}

template <typename T_>
ptime TimeLine<T_>::getNearestTime(const boost::shared_ptr<T_> &data)
{
    auto d = getNearestData(data->timestamp);
    if (d)
        return d->timestamp;
    else
        return ptime();
}

template <typename T_>
pair<ptime, ptime> TimeLine<T_>::getBoundaryTime(const ptime &t)
{
    auto p = getBoundaryData(t);
    ptime fore, back;
    if (p.first) fore = p.first->timestamp;
    if (p.second) back = p.second->timestamp;
    return make_pair(fore, back);
}

template <typename T_>
pair<ptime, ptime> TimeLine<T_>::getBoundaryTime(const boost::shared_ptr<T_ > &data)
{
    return getBoundaryTime(data->timestamp);
}

template <typename T_>
boost::shared_ptr<T_> TimeLine<T_>::getNearestData(const ptime &t, bool shouldSetCursor)
{
    if (timeline.empty()) // dataset not ready
        return boost::shared_ptr<T_>();
    // search in the buffer first
    if (shouldSetCursor) { // if need to set cursor, don't search in the buffer
        if (t >= maxTime) {
            return buffer.back();
        } else if (t == buffer.front()->timestamp) {
            return buffer.front();
        } else if (t > buffer.front()->timestamp) {
            boost::shared_ptr<T_> result = buffer.front();
            int index = 1;
            while (buffer[index]->timestamp < t) ++index;
            if (buffer[index]->timestamp == t){
                return buffer[index];
            } else {
                result = buffer[index - 1];
                boost::shared_ptr<T_> anotherPossible = buffer[index];
                if ((t - result->timestamp).total_microseconds()
                        >= (anotherPossible->timestamp - t).total_microseconds()) {
                    result = anotherPossible;
                }
            }
            return result;
        }
    }
    // if not found in the buffer, search in the timeline instead (slower)
    boost::shared_ptr<T_> result;
    if (t >= maxTime) {
        return timeline.back().back();
    }
    if (t <= startTime) {
        return timeline[0][0];
    }
    int index = (t - startTime).total_microseconds() / interval;
    if (timeline[index].size() != 0) {
        if (shouldSetCursor) {timelineCursor = index; interCursor = 0;} // most of the cases will be true
        result = timeline[index][0];
        if (t == result->timestamp) {
            return result;
        } else if (t < result->timestamp) {
            if (index > 0 && timeline[index - 1].size() > 0) {
                boost::shared_ptr<T_> anotherPossible = timeline[index - 1].back();
                if ((t - anotherPossible->timestamp) < (result->timestamp - t)) {
                    if (shouldSetCursor) {timelineCursor = index - 1; interCursor = timeline[index - 1].size() - 1;}
                    return anotherPossible;
                } else {
                    return result;
                }
            }
        } else { // t > timeline[index][0]->timestamp
            if (timeline[index].back()->timestamp < t) {
                if (timeline[index + 1].size() > 0) {
                    bool thisNotNext = (t - timeline[index].back()->timestamp) < (timeline[index + 1][0]->timestamp - t);
                    if (shouldSetCursor) {
                        timelineCursor = thisNotNext ? index : (index + 1);
                        interCursor = thisNotNext ? (timeline[index].size() - 1) : 0;
                    }
                    return thisNotNext ? timeline[index].back() : timeline[index + 1][0];
                } else {
                    if (shouldSetCursor) {
//                        timelineCursor = index;
                        interCursor = timeline[index].size() -1;
                    }
                    return timeline[index].back();
                }
            } else if (timeline[index].size() > 1){ // answer must lie within
                int cursor = timeline[index].size() - 1;
                while (timeline[index][cursor]->timestamp > t) --cursor;
                bool foreNotBack = (t - timeline[index][cursor]->timestamp) < (timeline[index][cursor + 1]->timestamp - t);
                if (shouldSetCursor) {
                    timelineCursor = index;
                    interCursor = foreNotBack ? cursor : cursor + 1;
                }
                return foreNotBack ? timeline[index][cursor] : timeline[index][cursor + 1];
            } else {
                return timeline[index][0];
            }
        }
    }
    // if all former condition does not met, search back and forth
    int back = (index--) + 1;
    while (timeline[back].empty()) ++back;
    while (timeline[index].empty()) --index;
    bool foreNotBack = (t - timeline[index].back()->timestamp) < (timeline[back][0]->timestamp - t);
    if (shouldSetCursor) {
        timelineCursor = foreNotBack? index : back;
        interCursor = foreNotBack ? (timeline[index].size() - 1) : 0;
    }
    return foreNotBack ? timeline[index].back() : timeline[back][0];
}

template <typename T_>
boost::shared_ptr<T_ > TimeLine<T_>::getNearestData(const boost::shared_ptr<T_ > &data)
{
    return getNearestData(data->timestamp);
}

template <typename T_>
pair<boost::shared_ptr<T_ >, boost::shared_ptr<T_ >> TimeLine<T_>::getBoundaryData(const ptime &t)
{
    if (timeline.empty()) { // dataset not ready
        return make_pair(boost::shared_ptr<T_>(), boost::shared_ptr<T_>());
    }
    boost::shared_ptr<T_> forward, backward;
    if (timeline.size() == 1) { // backward will be invalid, remember to check it before use!!!
        forward = timeline[0][0];
        return make_pair(forward, backward);
    }
    if (t <= startTime) { // return the foremost two items
        forward = timeline[0][0];
        if (timeline[0].size() > 1){
            backward = timeline[0][1];
        } else {
            backward = timeline[1][0];
        }
        return make_pair(forward, backward);
    }
    if (t >= maxTime) { // return the last two items
        backward = buffer.back();
        forward = buffer[buffer.size() - 2];
        return make_pair(forward, backward);
    }

    // after we have dealt with the >=/<= special cases, we can proceed with normal cases

    // as usual, search the buffer first
    if (t > buffer[0]->timestamp) {
        int index = 1;
        while (buffer[index]->timestamp < t) ++index;
        return make_pair(buffer[index - 1], buffer[index]);
    }
    int index = (t - startTime).total_microseconds() / interval;
    if (timeline[index].size() != 0) {
        if (timeline[index][0]->timestamp <= t) { // most complicated case
            forward = timeline[index][0];
            for (int i = 1; i < timeline[index].size(); ++i) {
                if (timeline[index][i]->timestamp < t) {
                    forward = timeline[index][i];
                } else {
                    backward = timeline[index][i];
                    break;
                }
            }
            if (! backward) { // means 'backward' had not been assigned a valid value
                int cursor = index + 1;
                while (cursor != timeline.size() && timeline[cursor].empty()) ++cursor;
                if (cursor != timeline.size())
                    backward = timeline[cursor].front();
            }
            if (timeline[index][0]->timestamp == t) { // there is still another possibility in forward direction
                int cursor = index - 1;
                while (cursor >=0 && timeline[cursor].empty()) --cursor;
                if (cursor != -1) {
                    boost::shared_ptr<T_> anotherPossible = timeline[cursor].back();
                    if (backward) {
                        time_duration diff_f = t - backward->timestamp;
                        time_duration diff_b = anotherPossible->timestamp - t;
                        if (diff_f > diff_b) {
                            backward = forward;
                            forward = anotherPossible;
                        }
                    } else {
                        backward = forward;
                        forward = anotherPossible;
                    }
                }
            }
        } else { // simpler case
            backward = timeline[index][0];
            int cursor = index - 1;
            while (cursor >=0 && timeline[cursor].empty()) --cursor;
            //if (cursor != -1) -- cursor couldn't be -1
            forward = timeline[cursor].back();
        }
    } else { // even simple case
        int cursor = index - 1;
        while (cursor >=0 && timeline[cursor].empty()) --cursor;
        forward = timeline[cursor].back();
        while (timeline[++index].empty()){/*empty*/}
        backward = timeline[index].front();
    }
    return make_pair(forward, backward);
}

template <typename T_>
pair<boost::shared_ptr<T_ >, boost::shared_ptr<T_ >> TimeLine<T_>::getBoundaryData(
        const boost::shared_ptr<T_ > &data)
{
    return getBoundaryData(data->timestamp);
}

template <typename T_>
vector<boost::shared_ptr<T_> > TimeLine<T_>::getRangeBetween(const ptime &a, const ptime &b)
{
    if (total_num == 0) return vector<boost::shared_ptr<T_> >();
    getNearestTime(a, true); // 'true' means set the cursors as well
    int indexa = timelineCursor, offseta = interCursor;
    int warden = indexa;
    getNearestTime(b, true);
    int indexb = timelineCursor, offsetb = interCursor;
    vector<boost::shared_ptr<T_> > result;
    result.insert(result.end(), timeline[indexa].begin() + offseta, timeline[indexa].end());
    while (++indexa != indexb) {
        if (!timeline[indexa].empty())
            result.insert(result.end(), timeline[indexa].begin(), timeline[indexa].end());
    }
    if (indexb != warden)
        result.insert(result.end(), timeline[indexb].begin(), timeline[indexb].begin() + offsetb);
    return std::move(result);
}


template <typename T_>
vector<boost::shared_ptr<T_> > TimeLine<T_>::getAll()
{
    if (total_num == 0) return vector<boost::shared_ptr<T_> >();
    int indexa = 0, offseta = 0;
    int indexb = timeline.size() - 1, offsetb = timeline[indexb].size() - 1;
    vector<boost::shared_ptr<T_> > result;
    result.insert(result.end(), timeline[indexa].begin() + offseta, timeline[indexa].end());
    while (++indexa != indexb) {
        if (!timeline[indexa].empty())
            result.insert(result.end(), timeline[indexa].begin(), timeline[indexa].end());
    }
    if (indexb != 0)
        result.insert(result.end(), timeline[indexb].begin(), timeline[indexb].begin() + offsetb);
    return std::move(result);
}

template <typename T_>
bool TimeLine<T_>::setCursorTo(ptime &t)
{
    if (t < startTime || t > maxTime) return false;
    this->getNearestTime(t, true);
    return true;
}

template <typename T_>
boost::shared_ptr<T_ > TimeLine<T_>::nextData()
{
    int index_ = timelineCursor, offset_ = interCursor;
    if (timelineCursor == (timeline.size() - 1) && interCursor == timeline.back().size() - 1) {
        // reset to the begining
        timelineCursor = interCursor = 0;
    }
    if (interCursor == (timeline[index_].size() - 1)) {
        ++ timelineCursor;
        interCursor = 0;
    } else {
        ++ interCursor;
    }
    return timeline[index_][offset_];
}

template <typename T_>
void TimeLine<T_>::calcNewAvgIntervalAndRearrange()
{
    interval = (maxTime - startTime).total_microseconds() / total_num;
    auto vec = getAll();
    timeline.clear();
    buffer.clear();
    for (int i = 0; i < vec.size(); ++i) {
        int index = (vec[i]->timestamp - startTime).total_microseconds() / interval;
        while (timeline.size() <= index) { // fill the timeline 'gap'
            timeline.push_back(std::vector<boost::shared_ptr<T_>>());
        }
        timeline[index].push_back(vec[i]);
        buffer.push_back(vec[i]);
    }
    resetCursor();
    intervalFinalized = true;
}

template <typename T_>
boost::shared_ptr<T_ > TimeLine<T_>::searchVec(const std::vector<boost::shared_ptr<T_ > > &vec
        , const ptime &t
        , long &shortest)
{
    boost::shared_ptr<T_ > result = vec[0];
    // type conversion to surpress warnings
    shortest = (t - result->timestamp).total_microseconds();
    if (shortest <= 0) {
        return result;
    }
    for (int i = 1; i < vec.size(); ++i) {
        long distance = abs((int)(t - vec[i]->timestamp).total_microseconds());
        if (distance < shortest) {
            result = vec[i];
            shortest = distance;
            return result;
        }
    }
    return result;
}

template <typename T_>
std::ifstream &operator>>(std::ifstream &is, TimeLine<T_> &obj) {
    boost::shared_ptr<T_> item(new T_());
    while (is >> *(item.get())) {
        obj.addData(item);
        item = boost::shared_ptr<T_>(new T_());
    }
    return is;
}

template <typename T_>
ofstream &operator<<(ofstream &os, TimeLine<T_>& obj)
{
    auto all = obj.getAll();
    for (auto & i : all) {
        os << *(i.get());
    }
    return os;
}

#endif // __TimeLine_H__
