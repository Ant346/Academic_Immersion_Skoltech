import re


def normalize_serial(value: str) -> str:
    return value.strip().rstrip(',').strip()


def _extract_section(text: str, section_name: str) -> str:
    pattern = rf"{re.escape(section_name)}\s*:\s*\{{"
    match = re.search(pattern, text)
    if not match:
        return ""

    start = match.end() - 1
    depth = 0
    for idx in range(start, len(text)):
        ch = text[idx]
        if ch == '{':
            depth += 1
        elif ch == '}':
            depth -= 1
            if depth == 0:
                return text[start + 1:idx]
    return ""


def _extract_number(section_text: str, key: str, cast):
    match = re.search(rf"{re.escape(key)}\s*=\s*([0-9.]+)", section_text)
    if not match:
        return None
    return cast(float(match.group(1))) if cast is int else cast(match.group(1))


def _extract_bool(section_text: str, key: str):
    match = re.search(rf"{re.escape(key)}\s*=\s*(true|false)", section_text, flags=re.IGNORECASE)
    if not match:
        return None
    return match.group(1).lower() == "true"


def parse_param_file(address: str) -> dict:
    with open(address, "r", encoding="utf-8") as f:
        text = f.read()

    result = {}

    resolution_section = _extract_section(text, "resolution")
    image_size_section = _extract_section(resolution_section, "image_size") if resolution_section else ""
    width = _extract_number(image_size_section, "iWidth", int) if image_size_section else None
    height = _extract_number(image_size_section, "iHeight", int) if image_size_section else None
    if width and height:
        result["resolution"] = f"{width},{height}"

    isp_color = _extract_section(text, "isp_color")
    if isp_color:
        invert_color = _extract_bool(isp_color, "inverse")
        saturation = _extract_number(isp_color, "saturation", int)
        monochrome = _extract_bool(isp_color, "mono")
        if invert_color is not None:
            result["invert_color"] = invert_color
        if saturation is not None:
            result["saturation"] = saturation
        if monochrome is not None:
            result["monochrome"] = monochrome

    isp_shape = _extract_section(text, "isp_shape")
    if isp_shape:
        sharpness = _extract_number(isp_shape, "sharpness", int)
        if sharpness is not None:
            result["sharpness"] = sharpness

    isp_lut = _extract_section(text, "isp_lut")
    if isp_lut:
        contrast = _extract_number(isp_lut, "contrast", int)
        gamma = _extract_number(isp_lut, "gamma", int)
        if contrast is not None:
            result["contrast"] = contrast
        if gamma is not None:
            result["gamma"] = gamma

    exposure = _extract_section(text, "exposure")
    if exposure:
        auto_exposure = _extract_bool(exposure, "ae_enable")
        auto_exposure_target = _extract_number(exposure, "ae_target", int)
        exposure_time = _extract_number(exposure, "user_exposure_time", int)
        analog_gain = _extract_number(exposure, "analog_gain", int)
        anti_flick_freq = _extract_number(exposure, "anti_flick_freq", int)
        antiflick = _extract_bool(exposure, "anti_flick")

        if auto_exposure is not None:
            result["auto_exposure"] = auto_exposure
        if auto_exposure_target is not None:
            result["auto_exposure_target"] = auto_exposure_target
        if exposure_time is not None:
            result["exposure_time"] = exposure_time
        if analog_gain is not None:
            result["analog_gain"] = analog_gain
        if anti_flick_freq is not None:
            result["light_frequency"] = 60 if anti_flick_freq else 50
        if antiflick is not None:
            result["antiflick"] = antiflick

    video_format = _extract_section(text, "video_format")
    if video_format:
        frame_speed = _extract_number(video_format, "frame_speed_sel", int)
        if frame_speed is not None:
            result["fps"] = (frame_speed + 1) * 10

    return result


def apply_param_file_overrides(data: dict) -> dict:
    address = data.get("GigE_param_file", "")
    if not address:
        return data

    overrides = parse_param_file(address)
    # Treat the SDK param file as a source of defaults only. Explicit values from
    # YAML should remain the source of truth for ROS/node behavior.
    merged = dict(overrides)
    merged.update(data)
    return merged
