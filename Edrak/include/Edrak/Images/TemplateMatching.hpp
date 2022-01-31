#ifndef __EDRAK_TEMPLATEMATCHING_H__
#define __EDRAK_TEMPLATEMATCHING_H__
#include "Edrak/Exceptions/Exceptions.hpp"
#include <opencv2/imgproc.hpp>
using cv::InputArray;
using cv::OutputArray;
namespace Edrak {
namespace Images {
/**
 * @brief
 *
 */
enum class TemplateMatchingMethod { CC, CC_NORMALIZED };
/**
 * @brief
 *
 * @param _img
 * @param _templ
 * @param _res
 * @param method
 */
void matchTemplate(InputArray _img, InputArray _templ, OutputArray _res,
                   TemplateMatchingMethod method);
} // namespace Images

} // namespace Edrak

#endif // __TEMPLATEMATCHING_H__