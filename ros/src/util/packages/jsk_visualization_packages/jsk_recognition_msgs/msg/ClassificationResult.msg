# information about frame and timestamp
Header header

# prediction results
uint32[] labels          # numerical labels
string[] label_names     # non-numerical labels
float64[] label_proba    # probabilities of labels
float64[] probabilities  # probabilities about each classification for all target_names

# information about classifier
string classifier        # name of classifier
string[] target_names    # set in which label_names should be
