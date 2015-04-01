#include "shape_loader.h"

#include <tue/filesystem/path.h>
#include <fstream>

#include <geolib/serialization.h>
#include <geolib/HeightMap.h>
#include <geolib/Shape.h>

#include <yaml-cpp/yaml.h>

#include <opencv2/highgui/highgui.hpp>

#include <geolib/Importer.h>

#include "xml_shape_parser.h"

// Heightmap generation
#include <opencv2/imgproc/imgproc.hpp>
#include <geolib/CompositeShape.h>

namespace ed
{
namespace models
{

// ----------------------------------------------------------------------------------------------------

void findContours(const cv::Mat& image, const geo::Vec2i& p, int d_start, std::vector<geo::Vec2i>& points,
                  std::vector<geo::Vec2i>& line_starts, cv::Mat& contour_map, bool add_first)
{
    static int dx[4] = {1,  0, -1,  0 };
    static int dy[4] = {0,  1,  0, -1 };

    unsigned char v = image.at<unsigned char>(p.y, p.x);

    int d_current = d_start; // Current direction
    int x2 = p.x;
    int y2 = p.y;

    int line_piece_min = 1e9; // minimum line piece length of current line
    int line_piece_max = 0; // maximum line piece length of current line

    int d_main = d_current; // The main direction in which we're heading. If we follow a line
                            // that gradually changes to the side (1-cell side steps), this direction
                            // denotes the principle axis of the line

    if (add_first)
        points.push_back(p - geo::Vec2i(1, 1));

    int n_uninterrupted = 1;
    geo::Vec2i p_corner = p;

    while (true)
    {
        bool found = false;
        int d = (d_current + 3) % 4; // check going left first

        for(int i = -1; i < 3; ++i)
        {
            if (image.at<unsigned char>(y2 + dy[d], x2 + dx[d]) == v)
            {
                found = true;
                break;
            }

            d = (d + 1) % 4;
        }

        if (!found)
            return;

        geo::Vec2i p_current(x2, y2);

        if ((d + 2) % 4 == d_current)
        {
            // 180 degree turn

            if (x2 == p.x && y2 == p.y) // Edge case: if we returned to the start point and
                                        // this is a 180 degree angle, return without adding it
                return;


            geo::Vec2i q = p_current;
            if (d == 0 || d_current == 0) // right
                --q.y;
            if (d == 3 || d_current == 3) // up
                --q.x;

            points.push_back(q);
            d_main = d;
            line_piece_min = 1e9;
            line_piece_max = 0;

        }
        else if (d_current != d_main)
        {
            // Not moving in main direction (side step)

            if (d != d_main)
            {
                // We are not moving back in the main direction
                // Add the corner to the list and make this our main direction

                points.push_back(p_corner);
                d_main = d_current;
                line_piece_min = 1e9;
                line_piece_max = 0;
            }
        }
        else
        {
            // Moving in main direction (no side step)

            if (d_current != d)
            {
                // Turning 90 degrees

                // Check if the length of the last line piece is OK w.r.t. the other pieces in this line. If it differs to much,
                // (i.e., the contour has taken a different angle), add the last corner to the list. This way, we introduce a
                // bend in the contour
                if (line_piece_max > 0 && (n_uninterrupted < line_piece_max - 2 || n_uninterrupted > line_piece_min + 2))
                {
                    // Line is broken, add the corner as bend
                    points.push_back(p_corner);

                    line_piece_min = 1e9;
                    line_piece_max = 0;
                }

                // Update the line piece lenth boundaries with the current found piece
                line_piece_min = std::min(line_piece_min, n_uninterrupted);
                line_piece_max = std::max(line_piece_max, n_uninterrupted);
            }
        }

        if (d_current != d)
        {
            geo::Vec2i q = p_current;
            if (d == 0 || d_current == 0) // right
                --q.y;
            if (d == 3 || d_current == 3) // up
                --q.x;

            p_corner = q;
            n_uninterrupted = 0;
        }

        if ((d_current == 3 && d != 2) || (d == 3 && d != 0)) // up
            line_starts.push_back(p_current);

        contour_map.at<unsigned char>(p_current.y, p_current.x) = 1;

        ++n_uninterrupted;

        if (points.size() > 1 && x2 == p.x && y2 == p.y)
            return;

        x2 = x2 + dx[d];
        y2 = y2 + dy[d];

        d_current = d;
    }
}

// ----------------------------------------------------------------------------------------------------

geo::ShapePtr loadShape(const std::string& model_path, tue::config::Reader cfg,
                        std::map<std::string, geo::ShapePtr>& shape_cache, std::stringstream& error)
{
    geo::ShapePtr shape;

    std::string path;
    if (cfg.value("path", path))
    {
        tue::filesystem::Path shape_path;

        if (model_path.empty())
            shape_path = path;
        else
            shape_path = model_path + "/" + path;

        // Check cache first
        std::map<std::string, geo::ShapePtr>::const_iterator it = shape_cache.find(shape_path.string());
        if (it != shape_cache.end())
            return it->second;

        if (shape_path.exists())
        {
            std::string xt = shape_path.extension();
            if (xt == ".geo")
            {
                geo::serialization::registerDeserializer<geo::Shape>();
                shape = geo::serialization::fromFile(shape_path.string());
            }
            else if (xt == ".3ds")
            {
                shape = geo::Importer::readMeshFile(shape_path.string());
            }
            else if (xt == ".xml")
            {
                std::string error;
                shape = parseXMLShape(shape_path.string(), error);
            }

            if (!shape)
                error << "ed::models::loadShape() : ERROR while loading shape at " << shape_path.string() << std::endl;
            else
                // Add to cache
                shape_cache[shape_path.string()] = shape;
        }
        else
        {
            error << "ed::models::loadShape() : ERROR while loading shape of at " << shape_path.string() << " ; file does not exist" << std::endl;
        }
    }
    else
    {
        error << "ed::models::loadShape() : ERROR while loading shape, no path specified in model.yaml" << std::endl;
    }

    return shape;
}

}
}
