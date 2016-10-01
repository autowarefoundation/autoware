#include "Context.h"

Context::Context(cv::Point aRedCenter, cv::Point aYellowCenter, cv::Point aGreenCenter,
		 int aLampRadius, cv::Point aTopLeft, cv::Point aBotRight)
{
	redCenter = aRedCenter;
	yellowCenter = aYellowCenter;
	greenCenter = aGreenCenter;
	lampRadius = aLampRadius;
	topLeft = aTopLeft;
	botRight = aBotRight;
}


/*
  define magnitude relationship of context
 */
bool Context::CompareContext(const Context left, const Context right)
{
  /* if lampRadius is bigger, context is smaller */
  return left.lampRadius >= right.lampRadius;
} /* static bool compareContext() */


void Context::SetContexts(std::vector<Context> &contexts,
                          const road_wizard::Signals::ConstPtr &extracted_pos,
                          const int frame_row,
                          const int frame_column) {
  /* copy parts of data to local variable */
  std::vector<road_wizard::ExtractedPosition> signals;
  std::vector<road_wizard::ExtractedPosition>::iterator sig_iterator;
  for (unsigned int i=0; i<extracted_pos->Signals.size(); i++ )
    {
      road_wizard::ExtractedPosition tmp;
      tmp.signalId = extracted_pos->Signals.at(i).signalId;
      tmp.u        = extracted_pos->Signals.at(i).u;
      tmp.v        = extracted_pos->Signals.at(i).v;
      tmp.radius   = extracted_pos->Signals.at(i).radius;
      tmp.x        = extracted_pos->Signals.at(i).x;
      tmp.y        = extracted_pos->Signals.at(i).y;
      tmp.z        = extracted_pos->Signals.at(i).z;
      tmp.type     = extracted_pos->Signals.at(i).type;
      tmp.linkId   = extracted_pos->Signals.at(i).linkId;
      tmp.plId     = extracted_pos->Signals.at(i).plId;
      signals.push_back(tmp);
    }

  std::vector<int> plid_vector;
  for (sig_iterator=signals.begin(); sig_iterator<signals.end(); sig_iterator++) {
    plid_vector.push_back(sig_iterator->plId);
  }

  /* get array that has unique PLID values as its element */
  std::sort(plid_vector.begin(), plid_vector.end());
  std::vector<int>::iterator new_end = std::unique(plid_vector.begin(), plid_vector.end());
  plid_vector.erase(new_end, plid_vector.end());

  std::vector<Context> updatedSignals;

  /* assemble fragmented signal lamp in a context */
  for (unsigned int ctx_idx=0; ctx_idx<plid_vector.size(); ctx_idx++)
    {
      Context ctx;
      int min_radius  = INT_MAX;
      int most_left   = frame_column;
      int most_top    = frame_row;
      int most_right  = 0;
      int most_bottom = 0;

      for (sig_iterator=signals.begin(); sig_iterator<signals.end(); sig_iterator++)
        {
          int img_x = sig_iterator->u;
          int img_y = sig_iterator->v;
          double map_x = sig_iterator->x;
          double map_y = sig_iterator->y;
          double map_z = sig_iterator->z;
          int radius = sig_iterator->radius;
          if (sig_iterator->plId == plid_vector.at(ctx_idx) &&
              0 < img_x - radius - 1.5 * radius && img_x + radius + 1.5 * radius < frame_column &&
              0 < img_y - radius - 1.5 * radius && img_y + radius + 1.5 * radius < frame_row)
            {
              switch (sig_iterator->type) {
              case 1:           /* RED */
                ctx.redCenter   = cv::Point( img_x, img_y );
                ctx.redCenter3d = cv::Point3d( map_x, map_y, map_z );
                break;
              case 2:           /* GREEN */
                ctx.greenCenter   = cv::Point( img_x, img_y );
                ctx.greenCenter3d = cv::Point3d( map_x, map_y, map_z );
                break;
              case 3:           /* YELLOW */
                ctx.yellowCenter   = cv::Point( img_x, img_y );
                ctx.yellowCenter3d = cv::Point3d( map_x, map_y, map_z );
                ctx.signalID       = sig_iterator->signalId; // use yellow light signalID as this context's representative
                break;
              default:          /* this signal is not for cars (for pedestrian or something) */
                continue;
              }
              min_radius    = (min_radius > radius) ? radius : min_radius;
              most_left     = (most_left > img_x - radius -   1.5 * min_radius)  ? img_x - radius - 1.5 * min_radius : most_left;
              most_top      = (most_top > img_y - radius -    1.5 * min_radius)  ? img_y - radius - 1.5 * min_radius : most_top;
              most_right    = (most_right < img_x + radius +  1.5 * min_radius)  ? img_x + radius + 1.5 * min_radius : most_right;
              most_bottom   = (most_bottom < img_y + radius + 1.5 * min_radius)  ? img_y + radius + 1.5 * min_radius : most_bottom;
            }
        }

      ctx.lampRadius = min_radius;
      ctx.topLeft    = cv::Point(most_left, most_top);
      ctx.botRight   = cv::Point(most_right, most_bottom);
      ctx.lightState = UNDEFINED;
      ctx.stateJudgeCount = 0;

      /* search whether this signal has already belonged in detector.contexts */
      bool isInserted = false;
      std::vector<int> eraseCandidate;
      for (unsigned int i = 0; i < contexts.size(); i++) {
        if (ctx.signalID == contexts.at(i).signalID && ctx.lampRadius != INT_MAX)
          {
            /* update to new information except to lightState */
            updatedSignals.push_back(ctx);
            updatedSignals.back().lightState      = contexts.at(i).lightState;
            updatedSignals.back().stateJudgeCount = contexts.at(i).stateJudgeCount;
            isInserted = true;
            break;
          }

      }

      if (isInserted == false && ctx.lampRadius != INT_MAX)
        updatedSignals.push_back(ctx); // this ctx is new in detector.contexts

    }

  /* sort by lampRadius */
  std::sort(updatedSignals.begin(), updatedSignals.end(), CompareContext);

  /* reset detector.contexts */
  contexts.clear();
  contexts.resize(updatedSignals.size());
  for (unsigned int i=0; i<updatedSignals.size(); i++) {
    contexts.at(i) = updatedSignals.at(i);
  }

} /* std::vector<Context> Context::SetContexts() */
