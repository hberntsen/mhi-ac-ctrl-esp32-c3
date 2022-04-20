idf_build_get_property(project_dir PROJECT_DIR)

idf_component_register(
    SRCS httpd.c wifi.c main.c
    INCLUDE_DIRS .
    EMBED_TXTFILES ${project_dir}/web/wifi.html.gz
)