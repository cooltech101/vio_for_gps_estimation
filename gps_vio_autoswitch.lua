-- Automatically switches between GPS and MAVLink Visual Pose / ExternalNav
-- Place in flight controller SD card root directory /APM/scripts/
--
-- Sources:
--   Source 1 = GPS
--   Source 2 = MAVLink Visual Pose / ExternalNav
--
-- No RC source selection switch is used.
-- No RC auto-enable switch is used.
--
-- EKF setup:
--   Configure EK3_SRC1_ parameters for GPS
--   Configure EK3_SRC2_ parameters for MAVLink Visual Pose / ExternalNav
--
-- Thresholds:
--   SCR_USER2 = GPS speed accuracy threshold
--   SCR_USER3 = ExternalNav vertical velocity innovation threshold
--
-- Example:
--   SCR_USER2 = 1.2
--   SCR_USER3 = 0.3
--
---@diagnostic disable: need-check-nil

local source_prev = 0                  -- defaults to Source 1 / GPS
local auto_switch = true               -- always automatically switch

local gps_usable_accuracy = 1.0        -- GPS is usable if speed accuracy is <= this value

local vote_counter_max = 20            -- 20 cycles at 100 ms = about 2 seconds
local gps_vs_extnav_vote = 0           -- -20 = GPS, +20 = ExternalNav


-- play tune on buzzer to alert user to change in active source set
function play_source_tune(source)
  if source ~= nil then
    if source == 0 then
      notify:play_tune("L8C")          -- Source 1 / GPS
    elseif source == 1 then
      notify:play_tune("L12DD")        -- Source 2 / ExternalNav
    end
  end
end


function update()

  -- check GPS speed accuracy threshold has been set
  local gps_speedaccuracy_thresh = param:get('SCR_USER2')

  if (gps_speedaccuracy_thresh == nil) or (gps_speedaccuracy_thresh <= 0) then
    gcs:send_text(0, "gps-extnav-source.lua: set SCR_USER2 to GPS speed accuracy threshold")
    return update, 1000
  end


  -- check ExternalNav innovation threshold has been set
  local extnav_innov_thresh = param:get('SCR_USER3')

  if (extnav_innov_thresh == nil) or (extnav_innov_thresh <= 0) then
    gcs:send_text(0, "gps-extnav-source.lua: set SCR_USER3 to ExtNav innovation threshold")
    return update, 1000
  end


  -- check GPS speed accuracy
  local gps_speed_accuracy = gps:speed_accuracy(gps:primary_sensor())

  local gps_over_threshold =
    (gps_speed_accuracy == nil) or
    (gps_speed_accuracy > gps_speedaccuracy_thresh)

  local gps_usable =
    (gps_speed_accuracy ~= nil) and
    (gps_speed_accuracy <= gps_usable_accuracy)


  -- get ExternalNav velocity innovations from AHRS
  -- source 6 corresponds to ExternalNav
  local extnav_innov = ahrs:get_vel_innovations_and_variances_for_source(6)

  local extnav_over_threshold =
    (extnav_innov == nil) or
    (extnav_innov:z() == 0.0) or
    (math.abs(extnav_innov:z()) > extnav_innov_thresh)

  local extnav_usable = not extnav_over_threshold


  -- voting logic
  --
  -- Vote towards GPS if:
  --   GPS is good, or
  --   GPS is still usable and ExternalNav is unusable
  --
  -- Vote towards ExternalNav if:
  --   GPS is poor and ExternalNav is usable
  if (not gps_over_threshold) or (gps_usable and not extnav_usable) then
    gps_vs_extnav_vote = math.max(gps_vs_extnav_vote - 1, -vote_counter_max)
  elseif extnav_usable then
    gps_vs_extnav_vote = math.min(gps_vs_extnav_vote + 1, vote_counter_max)
  end


  -- determine automatic source
  local auto_source = -1

  if gps_vs_extnav_vote <= -vote_counter_max then
    auto_source = 0                   -- Source 1 / GPS
  elseif gps_vs_extnav_vote >= vote_counter_max then
    auto_source = 1                   -- Source 2 / MAVLink Visual Pose / ExternalNav
  end


  -- automatic switching
  if auto_switch and (auto_source >= 0) and (auto_source ~= source_prev) then
    source_prev = auto_source
    ahrs:set_posvelyaw_source_set(source_prev)

    gcs:send_text(0, "Auto switched to Source " .. string.format("%d", source_prev + 1))
    play_source_tune(source_prev)
  end


  return update, 100
end


return update()
