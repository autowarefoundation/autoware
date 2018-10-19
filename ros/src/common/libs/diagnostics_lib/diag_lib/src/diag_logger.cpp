#include <diag_lib/diag_logger.h>

diag_logger::diag_logger()
{
    text_.action = text_.ADD;
    text_.width = 320;
    text_.height = 480;
    text_.top = 0;
    text_.left = 0;
    text_.fg_color.r = 0;
    text_.fg_color.g = 1;
    text_.fg_color.b = 1;
    text_.fg_color.a = 0.5;
    text_.bg_color.r = 0;
    text_.bg_color.g = 0;
    text_.bg_color.b = 0;
    text_.bg_color.a = 0.5;
    buffer_length_ = 10;
    buf_ = boost::circular_buffer<diag_msgs::diag>(buffer_length_);
}

diag_logger::~diag_logger()
{

}

void diag_logger::update(diag_msgs::diag data)
{
    std::vector<std::string> node_lists = filter_.get_node_lists();
    buf_.push_back(data);
    text_.text = "";
    std::vector<diag_info> found_diag_list;
    for(auto node_name_itr = node_lists.begin(); node_name_itr != node_lists.end(); node_name_itr++)
    {
        for(auto buf_itr = buf_.begin(); buf_itr != buf_.end(); buf_itr++)
        {
            /*
            boost::optional<diag_msgs::diag_node_errors> node_errors = filter_.filter(*buf_itr, *node_name_itr);
            if(node_errors && node_errors.get().errors.size() != 0)
            {
                for(auto error_itr = node_errors.get().errors.begin(); error_itr != node_errors.get().errors.end(); error_itr++)
                {
                    volatile bool is_founded = false;
                    for(auto found_diag_list_itr = found_diag_list.begin(); found_diag_list_itr != found_diag_list.end(); found_diag_list_itr++)
                    {
                        if(found_diag_list_itr->num == error_itr->num)
                        {
                            is_founded = true;
                        }
                    }
                    if(is_founded == false)
                    {
                        //diag_info info = diag_info();
                    }
                }
            }
            */
        }
    }
    return;
}