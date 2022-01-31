#include "Edrak/Images/TemplateMatching.hpp"
#include <boost/format.hpp>
#include <opencv2/core.hpp>
namespace Edrak {
namespace Images {
void matchTemplate(InputArray _img, InputArray _templ, OutputArray _res,
                   TemplateMatchingMethod method) {
  if (_templ.rows() > _img.rows() || _templ.cols() > _img.cols()) {
    throw Edrak::Exceptions::InvalidDimensions(
        (boost::format(
             "Template's size is bigger than the target image's size.\n "
             "Template(%1%,%2%), Image(%3%,%4%)") %
         _templ.rows() % _templ.cols() % _img.rows() % _img.cols())
            .str());
  }
  _res.create(_img.rows() - _templ.rows() + 1, _img.cols() - _templ.cols() + 1,
              CV_32FC1);
  cv::Mat _img_fl, _templ_fl;
  _img.getMat().convertTo(_img_fl, CV_32FC1);
  _templ.getMat().convertTo(_templ_fl, CV_32FC1);
  cv::filter2D(_img_fl, _res, -1, _templ_fl);
  cv::normalize(_res.getMat(), _res.getMat(), 0, 1, cv::NORM_MINMAX);
  // TODO Normalize the crossCorrelation
}
} // namespace Images

} // namespace Edrak
