import typing as t

from annotation_gui_gcp.lib.views.image_view import ImageView


class ControlPointFinderView(ImageView):
    def __init__(
        self,
        main_ui,
        web_app,
    ):
        super().__init__(main_ui, web_app, "/current_cp_view", [], False)

    def template_name(self):
        class_name = self.__class__.__base__.__name__
        return class_name

    def get_candidate_images(self) -> t.List[str]:
        images_in_existing_views = []
        for v in self.main_ui.sequence_views:
            images_in_existing_views.extend(v.image_list)

        point = self.main_ui.gcp_manager.points.get(self.main_ui.curr_point)
        images_this_point = (
            [obs.image_id for obs in point.observations] if point else []
        )

        # First list those images that are not in any other view
        images_this_point_unaccounted_for = [
            i for i in images_this_point if i not in images_in_existing_views
        ]
        images_this_point_other_views = [
            i for i in images_this_point if i in images_in_existing_views
        ]
        return images_this_point_unaccounted_for + images_this_point_other_views
