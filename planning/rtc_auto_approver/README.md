# RTC Auto Approver

## Purpose

RTC Auto Approver is a node to approve request to cooperate from behavior planning modules automatically.

## Inner-workings / Algorithms

```plantuml

start
:Receive RTC status;
if (Auto approver enabled?) then (yes)
  if (Status is safe && Current command is DEACTIVATE?) then (yes)
    :Send ACTIVATE;
    end
  endif
  if (Status is unsafe && Current command is ACTIVATE?) then (yes)
    :Send DEACTIVATE;
    end
  endif
endif
end

```

## Assumptions / Known limits

## Future extensions / Unimplemented parts
